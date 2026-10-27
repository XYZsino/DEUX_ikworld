from types import SimpleNamespace as NS
from deux_task_runtime.visual_selection import held_candidate


def candidate(identity, xyz, valid=True):
    return NS(target_id=identity,bag_visible=True,surface_center_valid=valid,
              surface_center=NS(x=xyz[0],y=xyz[1],z=xyz[2]))


def test_unique_raised_visible_candidate_near_measured_hand():
    held=candidate('new-image-id',[.5,0.,.9])
    floor=candidate('floor',[.5,0.,.72])
    assert held_candidate([held,floor],[.5,0.,.9],.72) is held
    assert held_candidate([floor],[.5,0.,.9],.72) is None
    assert held_candidate([held],[1.,0.,.9],.72) is None


def test_ambiguous_invalid_and_uninitialized_candidates_rejected():
    a=candidate('a',[.5,0.,.9]);b=candidate('b',[.51,0.,.9])
    assert held_candidate([a,b],[.5,0.,.9],.72) is None
    assert held_candidate([a],[.5,0.,.9],None) is None
    assert held_candidate([candidate('nan',[float('nan'),0.,.9])],[.5,0.,.9],.72) is None
    assert held_candidate([candidate('invalid',[.5,0.,.9],False)],[.5,0.,.9],.72) is None
