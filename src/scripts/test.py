from fr5_init import fr5robot

fr5robot = fr5robot(2)

# fr5robot.Go_to_start_zone()
fr5robot.point_safe_move([200, -450, 150, 90, 0, 0])