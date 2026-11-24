from robo_virtual_wall.nav_config import add_tape_filters


def test_operational_settings_preserved_with_tape_filters():
    base = dict(controller_server=dict(ros__parameters=dict(cmd_vel_topic='/cmd_vel_ctrl')))
    overlay = {}
    for name in ['local_costmap','global_costmap']:
        base[name] = {name: dict(ros__parameters=dict(plugins=['obstacle_layer','inflation_layer'],
            filters=['existing_filter'],resolution=.05,robot_base_frame='base_link'))}
        overlay[name] = {name: dict(ros__parameters=dict(filters=['tape_keepout_filter','tape_inflation'],
            tape_keepout_filter=dict(plugin='nav2_costmap_2d::KeepoutFilter'),
            tape_inflation=dict(plugin='nav2_costmap_2d::InflationLayer')))}
    result = add_tape_filters(base,overlay)
    assert result['controller_server'] == base['controller_server']
    for name in overlay:
        params = result[name][name]['ros__parameters']
        assert params['plugins'] == ['obstacle_layer','inflation_layer']
        assert params['filters'] == ['existing_filter','tape_keepout_filter','tape_inflation']
        assert base[name][name]['ros__parameters']['filters'] == ['existing_filter']
    assert add_tape_filters(result,overlay) == result
