"""PROTOTIPO: cumpre o objetivo sozinho assim que o mapeamento termina.

Fluxo:
  1. durante o mapeamento, o locais_node marca os locais e as anotacoes
     'objetos' (o que ha para pegar) e 'recebe' (o que vai para la);
  2. ~/finalizar_mapeamento no locais_node confere o plano e publica em
     /locais/mapeamento_concluido;
  3. este no recebe o aviso, monta o objetivo, planeja e executa, refazendo
     o plano quando algo falha.

Tambem da para comecar na mao (~/iniciar) ou so ver o plano (~/planejar).

modo:=simulado  -> nao precisa de robo nem de Nav2 (padrao);
modo:=nav2      -> anda de verdade (exige permitir_movimento:=true);
                   alinhar/pegar/soltar seguem simulados.

Durante a missao, a parada de emergencia do robo_hardware (/emergency_stop,
botao de emergencia do robo_controle) pausa a execucao ate ser liberada.
"""

from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from robo_locais.locais import MapaDeLocais
from robo_locais.missao import AcoesSimuladas, NavegadorSimulado, _ignorar
from robo_locais.missao_node import (
    NavegadorNav2,
    abrir_registro,
    criar_navegador,
    declarar_parametros_nav2,
)
from robo_locais.registro import RegistroMissao
from robo_locais.planejador import ExecutorObjetivo, obter_objetivo


class AutonomoNode(Node):
    def __init__(self) -> None:
        super().__init__("autonomo")
        self.declare_parameter("arquivo_locais", "~/robo_mapas/arena_locais.yaml")
        self.declare_parameter("arquivo_objetivo", "")  # vazio: usa 'recebe' dos locais
        self.declare_parameter("modo", "simulado")
        declarar_parametros_nav2(self)
        self.declare_parameter("capacidade", 1)
        self.declare_parameter("max_tentativas", 2)
        # ao_finalizar_mapeamento | imediato | manual (so pelo ~/iniciar)
        self.declare_parameter("iniciar", "ao_finalizar_mapeamento")
        self.declare_parameter("atraso_inicio_s", 3.0)

        self.pedido: Optional[str] = None  # arquivo de locais a executar
        self.executando = False
        self.navegador = None  # criado so na primeira execucao (espera o Nav2)

        self.create_subscription(
            String, "/locais/mapeamento_concluido", self.on_mapeamento_concluido, 10
        )
        self.create_service(Trigger, "~/iniciar", self.srv_iniciar)
        self.create_service(Trigger, "~/planejar", self.srv_planejar)

        quando = str(self.get_parameter("iniciar").value)
        if (str(self.get_parameter("modo").value) == "nav2"
                and not bool(self.get_parameter("permitir_movimento").value)):
            self.get_logger().warn(
                "permitir_movimento=false: no modo nav2 o robo NAO vai andar "
                "(use ~/planejar para conferir o plano, ou reinicie com "
                "permitir_movimento:=true)"
            )
        self.gatilho = ""  # o que disparou a missao, vai para o registro
        self.registro: Optional[RegistroMissao] = None
        if quando == "imediato":
            self.pedido, self.gatilho = self._arquivo_locais(), "imediato (ao subir o no)"
        self.get_logger().info(f"pronto (iniciar: {quando})")

    def _arquivo_locais(self) -> str:
        return str(self.get_parameter("arquivo_locais").value)

    # ---------- Gatilhos ----------

    def on_mapeamento_concluido(self, msg: String) -> None:
        if str(self.get_parameter("iniciar").value) != "ao_finalizar_mapeamento":
            self.get_logger().info("mapeamento concluido (iniciar manual, aguardando ~/iniciar)")
            return
        if self.executando:
            self.get_logger().warn("ja executando; aviso ignorado")
            return
        self.pedido = msg.data or self._arquivo_locais()
        self.gatilho = "finalizar_mapeamento"
        self.get_logger().info(f"mapeamento concluido, objetivo de {self.pedido}")

    def srv_iniciar(self, _req, resp):
        if self.executando or self.pedido:
            resp.success, resp.message = False, "ja executando"
            return resp
        self.pedido, self.gatilho = self._arquivo_locais(), "servico ~/iniciar"
        resp.success, resp.message = True, "objetivo sera iniciado"
        return resp

    def srv_planejar(self, _req, resp):
        """Plano em linha reta, sem mover o robo nem precisar do Nav2."""
        try:
            mapa = MapaDeLocais.carregar(self._arquivo_locais())
            plano = self._executor(mapa, NavegadorSimulado(lambda _m: None),
                                   lambda _m: None).planejar(self._objetivo(mapa))
        except (OSError, KeyError, ValueError) as erro:
            resp.success, resp.message = False, f"objetivo invalido: {erro}"
            return resp
        resp.success = not plano.problemas
        resp.message = plano.resumo()
        return resp

    # ---------- Execucao ----------

    def _objetivo(self, mapa: MapaDeLocais):
        return obter_objetivo(mapa, str(self.get_parameter("arquivo_objetivo").value))

    def _executor(self, mapa, navegador, log, ao_evento=_ignorar) -> ExecutorObjetivo:
        capacidade = int(self.get_parameter("capacidade").value)
        return ExecutorObjetivo(
            mapa, navegador, AcoesSimuladas(log, capacidade), log,
            capacidade=capacidade,
            max_tentativas=int(self.get_parameter("max_tentativas").value),
            ao_evento=ao_evento,
        )

    def _contexto(self, arquivo: str, mapa: MapaDeLocais, objetivo) -> dict:
        g = lambda nome: self.get_parameter(nome).value  # noqa: E731
        return {
            "tipo": "missao autonoma (autonomo_node)",
            "gatilho": self.gatilho or "?",
            "arquivo_locais": arquivo,
            "mapa": mapa.mapa,
            "locais": len(mapa.locais),
            "objetivo": str(g("arquivo_objetivo")) or "anotacoes 'recebe' dos locais",
            "entregas_pedidas": [f"{e.objeto} -> {e.para}" for e in objetivo.entregas],
            "terminar_em": objetivo.terminar_em,
            "modo": str(g("modo")),
            "permitir_movimento": bool(g("permitir_movimento")),
            "capacidade": int(g("capacidade")),
            "max_tentativas": int(g("max_tentativas")),
        }

    def _criar_navegador(self):
        return criar_navegador(
            self, str(self.get_parameter("modo").value), self.get_logger().info
        )

    def cancelar(self) -> None:
        if isinstance(self.navegador, NavegadorNav2):
            self.get_logger().warn("cancelando o objetivo do Nav2")
            self.navegador.cancelar()
        if self.registro is not None:
            self.registro.fechar(situacao="interrompida")  # nada se ja fechou

    def executar_pedido(self) -> None:
        arquivo, self.pedido = self.pedido, None
        self.executando = True
        log = self.get_logger().info
        self.registro = None
        try:
            mapa = MapaDeLocais.carregar(arquivo)
            objetivo = self._objetivo(mapa)
            self.registro = abrir_registro(
                self, "missao", self._contexto(arquivo, mapa, objetivo)
            )
            if not mapa.locais:
                self.get_logger().error(f"nenhum local em {arquivo}")
                self.registro.erro(f"nenhum local em {arquivo}")
                self.registro.fechar(situacao="nao_iniciada")
                return
            if self.navegador is None:
                try:
                    self.navegador = self._criar_navegador()
                except ValueError as erro:
                    self.get_logger().error(f"robo NAO vai andar: {erro}")
                    self.registro.erro(f"robo NAO vai andar: {erro}")
                    self.registro.fechar(situacao="nao_iniciada")
                    return
            if isinstance(self.navegador, NavegadorNav2):
                self.navegador.ao_evento = self.registro.evento
            executor = self._executor(mapa, self.navegador, log, self.registro.evento)

            plano = executor.planejar(objetivo)
            log("plano inicial:\n" + plano.resumo())
            self.registro.plano_inicial(plano)
            if not plano.acoes:
                self.get_logger().error("nada a fazer; confira os locais e o objetivo")
                self.registro.fechar(situacao="nao_iniciada")
                return

            atraso = float(self.get_parameter("atraso_inicio_s").value)
            if atraso > 0:
                log(f"robo comeca a andar sozinho em {atraso:.0f} s (Ctrl+C cancela)")
                time.sleep(atraso)

            relatorio = executor.executar(objetivo)
            for item in relatorio.entregues:
                log(f"  entregue: {item}")
            for item in relatorio.abandonadas:
                self.get_logger().warn(f"  nao entregue: {item}")
            self.registro.fechar(
                relatorio, situacao="concluida" if relatorio.sucesso else "incompleta"
            )
        except (OSError, KeyError, ValueError) as erro:
            self.get_logger().error(f"objetivo invalido: {erro}")
            if self.registro is not None:
                self.registro.erro(f"objetivo invalido: {erro}")
                self.registro.fechar(situacao="erro")
        finally:
            self.executando = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutonomoNode()
    try:
        # A missao roda aqui, fora dos callbacks, para os servicos seguirem
        # respondendo entre uma execucao e outra.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.pedido:
                node.executar_pedido()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.cancelar()
    except Exception as erro:
        if node.registro is not None:
            node.registro.erro(f"{type(erro).__name__}: {erro}")
            node.registro.fechar(situacao="erro")
        raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
