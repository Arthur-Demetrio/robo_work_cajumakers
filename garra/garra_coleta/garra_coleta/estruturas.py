"""Estruturas da arena (chao, workstations, estantes) como caixas de colisao do MoveIt.

As medidas ficam em config/bateria_virtual.yaml (frame robo_base, metros). Usado pelo pegar_cubo
(parametro estante) e pela bateria_virtual.
"""

import os

from geometry_msgs.msg import Pose
from moveit_msgs.msg import AllowedCollisionEntry, CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
import rclpy
from shape_msgs.msg import SolidPrimitive
import yaml

FRAME = 'robo_base'
PREFIXO = 'estrutura_'
# Pontas dos dedos. No modelo ficam ~2 cm abaixo do tool0; a garra real pega com a ponta no centro do
# cubo sem encostar na superficie (2026-11-24).
DEDOS = ('P_1', 'P_1_1')
# Links da mao (fixos abaixo de 'Base'). Pelo mesmo deslocamento do modelo, a mao toda pode tocar a
# superficie de APOIO do cubo (ACM). Na estante, sem isso a pre-pegada horizontal fica no limite de
# colisao e o OMPL nao amostra estados validos. Teto, borda e o resto do braco continuam proibidos.
MAO = ('Base', 'Motor', 'Engr_1', 'Engr_2', 'Junta', 'Junta_1', 'Junta_1_1', 'Junta_2') + DEDOS


def config_padrao():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('garra_coleta'), 'config', 'bateria_virtual.yaml')
    except Exception:
        return os.path.join(os.path.dirname(__file__), '..', 'config', 'bateria_virtual.yaml')


def caixa(nome, x0, x1, z0, z1, y, largura):
    o = CollisionObject()
    o.header.frame_id = FRAME
    o.id = PREFIXO + nome
    o.operation = CollisionObject.ADD
    s = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[x1 - x0, largura, z1 - z0])
    p = Pose()
    p.position.x, p.position.y, p.position.z = (x0 + x1) / 2, y, (z0 + z1) / 2
    p.orientation.w = 1.0
    o.primitives, o.primitive_poses = [s], [p]
    return o


def caixas_da_estrutura(e, y, largura):
    """(caixas de colisao, altura da superficie do cubo, nome do corpo dessa superficie)."""
    piso = []   # o chao ja e o robo_base
    x0, alt = e['borda_x'], e.get('altura', 0.0)
    if e['tipo'] == 'chao':
        return piso, 0.0, 'robo_base'
    x1 = x0 + e['profundidade']
    if e['tipo'] == 'mesa':
        return piso + [caixa('tampo', x0, x1, 0.0, alt, y, largura)], alt, PREFIXO + 'tampo'
    if e['tipo'] == 'estante':
        esp, teto = e['espessura'], alt + e['vao']
        return piso + [caixa('prateleira', x0, x1, 0.0, alt, y, largura),
                       caixa('teto', x0, x1, teto, teto + esp, y, largura)], alt, PREFIXO + 'prateleira'
    raise ValueError(f"tipo desconhecido: {e['tipo']}")


def dedos_na_superficie(superficie):
    """Pares (link da mao, superficie) a liberar na ACM; o chao (robo_base) fica como esta."""
    return [] if superficie == 'robo_base' else [(d, superficie) for d in MAO]


def carregar(caminho=None):
    with open(caminho or config_padrao()) as f:
        return yaml.safe_load(f)


def procurar(cfg, nome):
    for e in cfg['estruturas']:
        if e['nome'] == nome:
            return e
    raise ValueError(f"estrutura '{nome}' nao esta no YAML: "
                     + ', '.join(e['nome'] for e in cfg['estruturas']))


class Cena:
    """Poe e tira as caixas das estruturas na cena do move_group."""

    def __init__(self, no):
        self.no = no
        self._srv = no.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self._get = no.create_client(GetPlanningScene, '/get_planning_scene')
        self._na_cena = []
        self._acm_base = None   # ACM do move_group antes da primeira troca (SRDF)

    def _chamar(self, cliente, req):
        if not cliente.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f'{cliente.srv_name} indisponivel. O MoveIt esta rodando?')
        f = cliente.call_async(req)
        rclpy.spin_until_future_complete(self.no, f)
        if f.result() is None:
            raise RuntimeError(f'Falha ao chamar {cliente.srv_name}')
        return f.result()

    def _acm(self, permitir):
        """ACM inteira (a diff do MoveIt substitui a matriz toda) = base + pares permitidos."""
        if self._acm_base is None:
            req = GetPlanningScene.Request()
            req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            self._acm_base = self._chamar(self._get, req).scene.allowed_collision_matrix
        base = self._acm_base
        nomes = list(base.entry_names)
        m = [list(e.enabled) for e in base.entry_values]
        for par in permitir:
            for n in par:
                if n not in nomes:
                    nomes.append(n)
                    for linha in m:
                        linha.append(False)
                    m.append([False] * len(nomes))
            i, j = nomes.index(par[0]), nomes.index(par[1])
            m[i][j] = m[j][i] = True
        acm = type(base)()
        acm.entry_names = nomes
        acm.entry_values = [AllowedCollisionEntry(enabled=linha) for linha in m]
        acm.default_entry_names = list(base.default_entry_names)
        acm.default_entry_values = list(base.default_entry_values)
        return acm

    def trocar(self, objetos, permitir=()):
        """Troca as caixas na cena. permitir: pares (link, objeto) cujo contato e liberado."""
        cena = PlanningScene(is_diff=True)
        for nome in self._na_cena:
            r = CollisionObject(id=nome, operation=CollisionObject.REMOVE)
            r.header.frame_id = FRAME
            cena.world.collision_objects.append(r)
        cena.world.collision_objects += objetos
        if permitir or self._acm_base is not None:
            cena.allowed_collision_matrix = self._acm(permitir)
        if not self._chamar(self._srv, ApplyPlanningScene.Request(scene=cena)).success:
            raise RuntimeError('Falha ao aplicar a cena')
        self._na_cena = [o.id for o in objetos]
