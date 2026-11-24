import json
import cv2
import numpy as np
import pytest
from robo_virtual_wall import detector
from robo_virtual_wall.projection import load_calibration, project_segment


def test_red_wrap_and_tape_pattern():
    image = np.full((480, 640, 3), 255, np.uint8)
    for x in range(100, 500, 30):
        cv2.rectangle(image, (x, 300), (x + 14, 320), (0, 0, 255), -1)
    cfg = dict(h_min=170, h_max=10, s_min=120, s_max=255, v_min=80, v_max=255)
    mask, hsv = detector.mascara_fita(image, cfg)
    assert len(detector.detectar_paredes(detector.unir_tracejado(mask), mask, hsv)) == 1
    solid = np.full_like(image, 255)
    cv2.rectangle(solid, (100, 300), (500, 320), (0, 0, 255), -1)
    mask, hsv = detector.mascara_fita(solid, cfg)
    assert not detector.detectar_paredes(detector.unir_tracejado(mask), mask, hsv)
    values = np.array([[[179, 255, 255], [0, 255, 255], [90, 255, 255]]], np.uint8)
    assert detector.mascara_cor(values, cfg).tolist() == [[255, 255, 0]]


def test_projection_validation(tmp_path):
    file = tmp_path / 'calibration.json'
    file.write_text(json.dumps({'H': np.eye(3).tolist()}))
    with pytest.raises(ValueError):
        load_calibration(file)
    data = dict(H=[[.01, 0, 0], [0, .01, 0], [0, 0, 1]],
                frame_id='base_link', units='m', image_space='raw', validated=True,
                image_size=[640, 480], valid_pixels=[[0,0],[639,0],[639,479],[0,479]])
    file.write_text(json.dumps(data))
    calibration = load_calibration(file)
    assert np.allclose(project_segment([[10,10],[20,20]], calibration, (640,480), 3),
                       [[.1,.1],[.2,.2]])
    assert project_segment([[-1,10],[20,20]], calibration, (640,480), 3) is None
    assert project_segment([[400,10],[500,20]], calibration, (640,480), 3) is None
    with pytest.raises(ValueError):
        project_segment([[10,10],[20,20]], calibration, (1280,960), 3)
    data['H'] = [[1,0,0],[0,1,0],[0,0,0]]
    file.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load_calibration(file)


def test_connected_chevrons_and_yellow_rejection():
    cfg = dict(h_min=170, h_max=10, s_min=60, s_max=255, v_min=40, v_max=255)
    for color, expected in [((0, 0, 255), True), ((0, 255, 255), False)]:
        image = np.full((480, 640, 3), 255, np.uint8)
        for x in range(100, 460, 30):
            pts = np.array([[x,200],[x+30,230],[x,260],
                            [x+12,260],[x+42,230],[x+12,200]], np.int32)
            cv2.fillPoly(image, [pts], color)
        # A connecting red edge defeats connected-component stripe counting.
        cv2.line(image, (100,200), (460,200), color, 4)
        mask, hsv = detector.mascara_fita(image, cfg)
        walls = detector.detectar_paredes(detector.unir_tracejado(mask), mask, hsv)
        assert bool(walls) == expected


def test_wide_diagonal_band_and_solid_red():
    cfg = dict(h_min=170, h_max=10, s_min=60, s_max=255, v_min=40, v_max=255)
    for striped in [True, False]:
        image = np.full((480,640,3), 255, np.uint8)
        if striped:
            for x in range(100,460,30):
                pts = np.array([[x,200],[x+50,300],[x+64,300],[x+14,200]], np.int32)
                cv2.fillPoly(image,[pts],(0,0,255))
        else:
            cv2.rectangle(image,(100,200),(510,300),(0,0,255),-1)
        mask,hsv = detector.mascara_fita(image,cfg)
        walls = detector.detectar_paredes(detector.unir_tracejado(mask),mask,hsv)
        assert bool(walls) == striped


@pytest.mark.parametrize('diagonal', [False, True])
def test_nearby_bands_keep_both_directions(diagonal):
    image = np.full((480,640,3),255,np.uint8)
    for x in range(60,550,25):
        cv2.rectangle(image,(x,160),(x+12,182),(0,0,255),-1)
    # A second striped band touches/approaches the first after closing.
    vertical = np.full_like(image,255)
    for y in range(185,380,22):
        cv2.rectangle(vertical,(320,y),(344,y+11),(0,0,255),-1)
    if diagonal:
        transform = cv2.getRotationMatrix2D((332,180),-35,1)
        vertical = cv2.warpAffine(vertical,transform,(640,480),borderValue=(255,255,255))
    image = np.minimum(image,vertical)
    cfg = dict(h_min=170,h_max=10,s_min=60,s_max=255,v_min=40,v_max=255)
    mask,hsv = detector.mascara_fita(image,cfg)
    walls = detector.detectar_paredes(detector.unir_tracejado(mask),mask,hsv)
    assert len(walls) == 2, walls
    angles = [np.deg2rad(w[2]) for w in walls]
    assert any(abs(np.sin(a-b)) > .5 for a in angles for b in angles), walls
    # Regression: merged groups used to return numpy.int64 endpoints, crashing
    # the ROS status publisher only when a junction was detected.
    payload = [dict(pixels=[list(p1), list(p2)], angle_deg=float(angle))
               for p1, p2, angle in walls]
    assert json.loads(json.dumps(payload))


def test_smoothing_matches_direction_and_drops_missing():
    from robo_virtual_wall.stabilization import SegmentSmoother
    smoother = SegmentSmoother()
    smoother.update([((10,20),(110,20),0)])
    result = smoother.update([((112,22),(12,22),180)])
    assert 20 <= result[0][0][1] < 22
    assert smoother.update([]) == []
    assert smoother.previous == []


def test_duplicate_edges_suppressed_without_losing_branch_or_parallel_band():
    segments = [((50,100),(600,100),0),
                ((420,90),(590,110),7),
                ((50,112),(600,112),0),
                ((400,100),(400,300),90),
                ((50,160),(600,160),0)]
    result = detector.consolidar_segmentos(segments)
    assert len(result) == 3
    assert ((400,100),(400,300),90) in result
    assert ((50,160),(600,160),0) in result


def test_front_preview_cannot_be_used_as_validated_base_link(tmp_path):
    data = dict(H=[[.01,0,0],[0,.01,0],[0,0,1]], frame_id='virtual_wall_front',
                units='m', image_space='raw', validated=False, image_size=[640,480],
                valid_pixels=[[0,0],[639,0],[639,479],[0,479]])
    file = tmp_path/'preview.json'
    file.write_text(json.dumps(data))
    assert load_calibration(file, preview=True)
    with pytest.raises(ValueError):
        load_calibration(file)


def test_centreline_distance_is_not_minimum_forward_distance():
    from robo_virtual_wall.projection import centreline_intersection
    # Pixel (u,v) maps to forward=.01u and lateral=.01v-.5.
    calibration = (np.array([[.01,0,0],[0,.01,-.5],[0,0,1.]]), (100,100),
                   np.float32([[0,0],[99,0],[99,99],[0,99]]))
    x, pixel = centreline_intersection(((30,20),(42,80)),calibration,(100,100))
    assert x == pytest.approx(.36)
    assert pixel == (36,50)
    assert centreline_intersection(((30,20),(42,40)),calibration,(100,100)) is None
    assert centreline_intersection(((30,50),(42,50)),calibration,(100,100)) is None
    assert centreline_intersection(((30,20),(30,80)),calibration,(100,100))[0] == pytest.approx(.30)
    narrow = (*calibration[:2],np.float32([[0,0],[99,0],[99,40],[0,40]]))
    assert centreline_intersection(((30,20),(42,80)),narrow,(100,100)) is None
    with pytest.raises(ValueError):
        centreline_intersection(((30,20),(42,80)),calibration,(200,200))


def test_preview_longitudinal_outside_and_transverse():
    from robo_virtual_wall.projection import preview_observation
    cal = (np.array([[.01,0,0],[0,.01,-.5],[0,0,1.]]), (100,100),
           np.float32([[20,20],[80,20],[80,80],[20,80]]))
    assert preview_observation(((10,49),(90,51)),cal,(100,100))['kind'] == 'longitudinal'
    assert preview_observation(((10,10),(90,10)),cal,(100,100))['kind'] == 'outside'
    result = preview_observation(((40,0),(40,99)),cal,(100,100))
    assert result['kind'] == 'crossing'
    assert result['forward_m'] == pytest.approx(.4)
    assert preview_observation(((40,20),(40,40)),cal,(100,100))['kind'] == 'no_crossing'
    # Polygon winding must not affect clipping.
    reverse = (*cal[:2],cal[2][::-1])
    assert preview_observation(((40,0),(40,99)),reverse,(100,100)) == result


def test_preview_filter_clears_loss_and_follows_steps():
    from robo_virtual_wall.stabilization import PreviewDistanceFilter
    f = PreviewDistanceFilter()
    for value in [.35,.351,.349]:
        result = f.update(value)
    assert result == pytest.approx(.35)
    assert f.update(.25) == pytest.approx(.25)
    assert f.update(None) is None
    assert f.update(.28) == pytest.approx(.28)


def test_collinear_corridors_use_full_validated_band(monkeypatch):
    image = np.full((480,640,3),255,np.uint8)
    for x in range(60,580,25):
        cv2.rectangle(image,(x,200),(x+12,220),(0,0,255),-1)
    cfg = dict(h_min=170,h_max=10,s_min=60,s_max=255,v_min=40,v_max=255)
    mask,hsv = detector.mascara_fita(image,cfg)
    # Validated corridors describe only the right-hand part of one full band.
    monkeypatch.setattr(detector, 'segmentos_do_grupo', lambda *args: [
        ((300,210),(440,210),0), ((460,210),(570,210),0)])
    walls = detector.detectar_paredes(detector.unir_tracejado(mask),mask,hsv)
    assert len(walls) == 1
    xs = sorted([walls[0][0][0],walls[0][1][0]])
    assert xs[0] < 80 and xs[1] > 560


def test_directional_closing_bridges_thin_stripes_without_widening_band():
    mask = np.zeros((100,300),np.uint8)
    for x in range(30,260,40):
        mask[45:51,x:x+10] = 255
    united = detector.unir_tracejado(mask)
    assert united[47,50] == 255
    assert not united[30].any()
    assert not united[65].any()


def test_join_short_collinear_gaps_preserves_parallel_and_branch():
    fragments = [((30,100),(200,100),0),((225,102),(400,102),0),
                 ((30,130),(400,130),0),((200,100),(200,250),90),
                 ((460,100),(600,100),0)]
    result = detector.unir_segmentos_alinhados(fragments)
    assert len(result) == 4
    assert any(s[0] == (30,100) and s[1] == (400,102) for s in result)
    assert ((30,130),(400,130),0) in result
    assert ((200,100),(200,250),90) in result


def test_real_faded_tape_reaches_left_end(monkeypatch):
    from pathlib import Path
    image = cv2.imread(str(Path(__file__).parent/'fixtures/faded_tape.png'))
    monkeypatch.setattr(detector,'ROI_TOPO',0.)
    cfg = dict(h_min=170,h_max=10,s_min=45,s_max=255,v_min=40,v_max=255)
    mask,hsv = detector.mascara_fita(image,cfg)
    walls = detector.detectar_paredes(detector.unir_tracejado(mask),mask,hsv)
    assert len(walls) == 1
    xs = sorted([walls[0][0][0],walls[0][1][0]])
    assert xs[0] < 90 and xs[1] > 625


def test_thin_longitudinal_tape_uses_single_centre_axis(monkeypatch):
    def competing_edges(*args):
        raise AssertionError('Thin validated tape must not use Hough stripe edges')
    monkeypatch.setattr(detector,'segmentos_do_grupo',competing_edges)
    cfg = dict(h_min=170,h_max=10,s_min=45,s_max=255,v_min=40,v_max=255)
    centres = []
    for brightness in [150,180,210,240]:
        image = np.full((480,640,3),240,np.uint8)
        for y in range(80,440,24):
            cv2.rectangle(image,(310,y),(330,y+12),(0,0,brightness),-1)
        mask,hsv = detector.mascara_fita(image,cfg)
        walls = detector.detectar_paredes(detector.unir_tracejado(mask),mask,hsv)
        assert len(walls) == 1
        p1,p2,_ = walls[0]
        assert abs(p2[0]-p1[0]) <= 2
        centres.append((p1[0]+p2[0])/2)
    assert max(centres)-min(centres) <= 2


@pytest.mark.parametrize('name',['longitudinal_tape_clean.png','longitudinal_tape_robot_wire.png'])
def test_real_longitudinal_tape_excludes_robot_red_wire(name,monkeypatch):
    from pathlib import Path
    image = cv2.imread(str(Path(__file__).parent/'fixtures'/name))
    monkeypatch.setattr(detector,'ROI_TOPO',0.)
    cfg = dict(h_min=170,h_max=10,s_min=45,s_max=255,v_min=40,v_max=255)
    mask,hsv = detector.mascara_fita(image,cfg)
    bottom = int(image.shape[0]*.85)
    mask[bottom:] = 0
    united = detector.unir_tracejado(mask)
    united[bottom:] = 0
    walls = detector.detectar_paredes(united,mask,hsv)
    assert len(walls) == 1,walls
    p1,p2,_ = walls[0]
    assert abs(p2[0]-p1[0]) <= 3
    assert max(p1[1],p2[1]) < bottom
    assert abs((p1[0]+p2[0])/2-274) <= 2


@pytest.mark.parametrize('name',['two_real_tapes_a.png','two_real_tapes_b.png'])
def test_real_two_tapes_preserve_both_branches_without_robot_wire(name,monkeypatch):
    from pathlib import Path
    image = cv2.imread(str(Path(__file__).parent/'fixtures'/name))
    monkeypatch.setattr(detector,'ROI_TOPO',0.)
    cfg = dict(h_min=170,h_max=10,s_min=45,s_max=255,v_min=40,v_max=255)
    mask,hsv = detector.mascara_fita(image,cfg)
    bottom = int(image.shape[0]*.85)
    mask[bottom:] = 0
    united = detector.unir_tracejado(mask)
    united[bottom:] = 0
    walls = detector.detectar_paredes(united,mask,hsv)
    assert len(walls) == 2,walls
    axes = [np.subtract(p2,p1).astype(float) for p1,p2,_ in walls]
    axes = [axis/np.linalg.norm(axis) for axis in axes]
    assert abs(axes[0]@axes[1]) < .2
    assert all(max(p1[1],p2[1]) < bottom for p1,p2,_ in walls)


def test_real_junction_axes_are_consistent_between_saved_frames(monkeypatch):
    from pathlib import Path
    from robo_virtual_wall.projection import load_calibration,project_observed_segment
    root = Path(__file__).parents[1]
    cal = load_calibration(root/'calibration/front_base_link_validated.json')
    monkeypatch.setattr(detector,'ROI_TOPO',0.)
    lateral,forward = [],[]
    for name in ['two_real_tapes_a.png','two_real_tapes_b.png']:
        image = cv2.imread(str(root/'test/fixtures'/name))
        cfg = dict(h_min=170,h_max=10,s_min=45,s_max=255,v_min=40,v_max=255)
        mask,hsv = detector.mascara_fita(image,cfg)
        mask[408:] = 0
        united = detector.unir_tracejado(mask)
        united[408:] = 0
        walls = detector.detectar_paredes(united,mask,hsv)
        assert len(walls) == 2
        for p1,p2,angle in walls:
            ground = project_observed_segment((p1,p2),cal,(640,480),3.)
            if abs(angle) > 45:
                t = (.70-ground[0,0])/(ground[1,0]-ground[0,0])
                lateral.append(ground[0,1]+t*(ground[1,1]-ground[0,1]))
            else:
                t = -ground[0,1]/(ground[1,1]-ground[0,1])
                forward.append(ground[0,0]+t*(ground[1,0]-ground[0,0]))
    assert len(lateral) == len(forward) == 2
    assert abs(lateral[0]-lateral[1]) < .005
    assert abs(forward[0]-forward[1]) < .005
