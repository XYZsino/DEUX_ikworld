"""The removed seven-second forecast must not return to RGB metadata."""
from test_rgb_view import fixture
from deux_task_gui.rgb_view import render_rgb

def test_no_seven_second_forecast_in_rgb():
 obs,_=fixture();jpeg,meta=render_rgb(obs,None,'')
 assert jpeg and not any(k.startswith('future_path') for k in meta)
