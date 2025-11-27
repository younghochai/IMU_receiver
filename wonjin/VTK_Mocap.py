from vtkmodules.vtkRenderingCore import (
    vtkRenderWindow,
    vtkRenderWindowInteractor,
    vtkRenderer,
)
from vtk import vtkTransform
from vtkmodules.vtkCommonColor import vtkNamedColors


from src.temp import TimerCallback

from src.utils import (
    make_cube_source,
    make_sphere_source,
    make_cylinder_source,
    make_mapper_actor,
)

from src.data import (
    start_data_thread,
    keypress_callback
)


def main():
    start_data_thread()

    colors = vtkNamedColors()

    renderer = vtkRenderer()
    renderer.SetBackground(colors.GetColor3d("White"))

    render_win = vtkRenderWindow()
    render_win.SetWindowName("ROBOT 7ea Sensors")
    render_win.SetSize(600, 800)
    render_win.AddRenderer(renderer)

    ren_win_interactor = vtkRenderWindowInteractor()

    lower_body_source = make_cylinder_source(11, 9)
    left_pelvis_source = make_sphere_source(4.7)
    right_pelvis_source = make_sphere_source(4.7)

    left_leg_source = make_cylinder_source(4.3, 14)
    right_leg_source = make_cylinder_source(4.3, 14)

    left_knee_source = make_sphere_source(4)
    right_knee_source = make_sphere_source(4)

    left_calf_source = make_cylinder_source(3, 13)
    right_calf_source = make_cylinder_source(3, 13)

    left_ankle_source = make_sphere_source(3)
    right_ankle_source = make_sphere_source(3)

    left_foot_source = make_cube_source(6, 3, 12)
    right_foot_source = make_cube_source(6, 3, 12)

    # 허리
    lower_body_transform = vtkTransform()
    lower_body_actor = make_mapper_actor(lower_body_source, lower_body_transform)

    # 골반
    left_pelvis_transform = vtkTransform()
    left_pelvis_transform.Translate(-5, -5, 0)
    left_pelvis_actor = make_mapper_actor(left_pelvis_source, left_pelvis_transform)

    right_pelvis_transform = vtkTransform()
    right_pelvis_transform.Translate(5, -5, 0)
    right_pelvis_actor = make_mapper_actor(right_pelvis_source, right_pelvis_transform)

    # 허벅지
    left_leg_transform = vtkTransform()
    left_leg_transform.Translate(0, -10, 0)
    left_leg_transform.SetInput(left_pelvis_transform)
    left_leg_actor = make_mapper_actor(left_leg_source, left_leg_transform)

    right_leg_transform = vtkTransform()
    right_leg_transform.Translate(0, -10, 0)
    right_leg_transform.SetInput(right_pelvis_transform)
    right_leg_actor = make_mapper_actor(right_leg_source, right_leg_transform)

    # 무릎
    left_knee_transform = vtkTransform()
    left_knee_transform.Translate(0, -9, 0)
    left_knee_transform.SetInput(left_leg_transform)
    left_knee_actor = make_mapper_actor(left_knee_source, left_knee_transform)

    right_knee_transform = vtkTransform()
    right_knee_transform.Translate(0, -9, 0)
    right_knee_transform.SetInput(right_leg_transform)
    right_knee_actor = make_mapper_actor(right_knee_source, right_knee_transform)

    # 종아리
    left_calf_transform = vtkTransform()
    left_calf_transform.SetInput(left_knee_transform)
    left_calf_actor = make_mapper_actor(left_calf_source, left_calf_transform)

    right_calf_transform = vtkTransform()
    right_calf_transform.SetInput(right_knee_transform)
    right_calf_actor = make_mapper_actor(right_calf_source, right_calf_transform)

    # 발목
    left_ankle_transform = vtkTransform()
    left_ankle_transform.Translate(0, -8, 0)
    left_ankle_transform.SetInput(left_calf_transform)
    left_ankle_actor = make_mapper_actor(left_ankle_source, left_ankle_transform)

    right_ankle_transform = vtkTransform()
    right_ankle_transform.Translate(0, -8, 0)
    right_ankle_transform.SetInput(right_calf_transform)
    right_ankle_actor = make_mapper_actor(right_ankle_source, right_ankle_transform)

    # 발
    left_foot_transform = vtkTransform()
    left_foot_transform.Translate(0, -3, 0)
    left_foot_transform.SetInput(left_ankle_transform)
    left_foot_actor = make_mapper_actor(left_foot_source, left_foot_transform)
    left_foot_actor.GetProperty().SetColor(colors.GetColor3d("SkyBlue"))

    right_foot_transform = vtkTransform()
    right_foot_transform.Translate(0, -3, 0)
    right_foot_transform.SetInput(right_ankle_transform)
    right_foot_actor = make_mapper_actor(right_foot_source, right_foot_transform)

    for actor in [
        lower_body_actor,
        left_pelvis_actor, right_pelvis_actor,
        left_leg_actor, right_leg_actor,
        left_knee_actor, right_knee_actor,
        left_calf_actor, right_calf_actor,
        left_ankle_actor, right_ankle_actor,
        left_foot_actor, right_foot_actor,
    ]:
        renderer.AddActor(actor)

    pelvis_cb = TimerCallback(
        lower_body_actor, lower_body_transform, 0, 0, render_win, 0
    )
    # 왼쪽
    left_leg_cb = TimerCallback(
        left_leg_actor, left_leg_transform, 0, -11, render_win, 1
    )
    left_calf_cb = TimerCallback(
        left_calf_actor, left_calf_transform, 0, -10, render_win, 2
    )
    left_foot_cb = TimerCallback(
        left_foot_actor, left_foot_transform, 0, -4, render_win, 3
    )
    # 오른쪽
    right_leg_cb = TimerCallback(
        right_leg_actor, right_leg_transform, 0, -11, render_win, 4
    )
    right_calf_cb = TimerCallback(
        right_calf_actor, right_calf_transform, 0, -10, render_win, 5
    )
    right_foot_cb = TimerCallback(
        right_foot_actor, right_foot_transform, 0, -4, render_win, 6
    )

    ren_win_interactor.AddObserver("TimerEvent", pelvis_cb)
    ren_win_interactor.AddObserver("TimerEvent", left_leg_cb)
    ren_win_interactor.AddObserver("TimerEvent", left_calf_cb)
    ren_win_interactor.AddObserver("TimerEvent", left_foot_cb)
    ren_win_interactor.AddObserver("TimerEvent", right_leg_cb)
    ren_win_interactor.AddObserver("TimerEvent", right_calf_cb)
    ren_win_interactor.AddObserver("TimerEvent", right_foot_cb)
    ren_win_interactor.AddObserver("KeyPressEvent", keypress_callback)

    ren_win_interactor.SetRenderWindow(render_win)
    ren_win_interactor.Initialize()
    ren_win_interactor.CreateRepeatingTimer(10)

    render_win.Render()
    ren_win_interactor.Start()


if __name__ == "__main__":
    main()
