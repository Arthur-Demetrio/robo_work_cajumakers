"""Leitura de MCAP do rosbag2 sem ROS: registra os tipos a partir dos schemas embutidos."""
from mcap.reader import make_reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from rosbags.typesys.msg import get_types_from_msg as _g  # noqa


def _split_concat(name, text):
    """Schemas ros2msg concatenados: blocos separados por linhas de '=' + 'MSG: pkg/Type'."""
    out = {}
    cur_name, buf = name, []
    for line in text.splitlines():
        if line.startswith('=' * 10):
            out[cur_name] = '\n'.join(buf)
            cur_name, buf = None, []
            continue
        if cur_name is None and line.startswith('MSG: '):
            cur_name = line[5:].strip()
            continue
        buf.append(line)
    if cur_name:
        out[cur_name] = '\n'.join(buf)
    return out


def _norm(n):
    p = n.split('/')
    return n if len(p) == 3 else f'{p[0]}/msg/{p[1]}'


def open_bag(path, topics=None):
    ts = get_typestore(Stores.ROS2_JAZZY)
    f = open(path, 'rb')
    r = make_reader(f)
    summ = r.get_summary()
    add = {}
    for sch in summ.schemas.values():
        for n, t in _split_concat(sch.name, sch.data.decode()).items():
            n = _norm(n)
            if n in ts.types:
                continue
            try:
                add.update(get_types_from_msg(t, n))
            except Exception:
                pass
    # registra iterativamente (dependências)
    for _ in range(5):
        pend = {k: v for k, v in add.items() if k not in ts.types}
        if not pend:
            break
        for k, v in pend.items():
            try:
                ts.register({k: v})
            except Exception:
                pass
    chans = {c.id: c for c in summ.channels.values()}
    schs = summ.schemas

    def it():
        for sch, ch, msg in r.iter_messages(topics=topics):
            yield ch.topic, msg.log_time * 1e-9, msg.data, _norm(sch.name)

    return ts, it


def deser(ts, data, typ):
    return ts.deserialize_cdr(data, typ)
