"""Add tape filters while preserving the operational Nav2 configuration."""
import copy


def add_tape_filters(base, overlay):
    result = copy.deepcopy(base)
    for name in ('local_costmap','global_costmap'):
        params = result[name][name]['ros__parameters']
        tape = overlay[name][name]['ros__parameters']
        filters = list(params.get('filters',[]))
        for key in tape['filters']:
            if key not in filters:
                filters.append(key)
            params[key] = copy.deepcopy(tape[key])
        params['filters'] = filters
    return result
