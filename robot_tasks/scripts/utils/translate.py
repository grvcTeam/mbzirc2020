import mbzirc_comm_objs.msg as msg


def color_from_int(color):
    if color == msg.ObjectDetection.COLOR_RED:
        return 'red'
    elif color == msg.ObjectDetection.COLOR_GREEN:
        return 'green'
    elif color == msg.ObjectDetection.COLOR_BLUE:
        return 'blue'
    elif color == msg.ObjectDetection.COLOR_ORANGE:
        return 'orange'
    elif color == msg.ObjectDetection.COLOR_UNKNOWN:
        return 'unknown'
    else:
        return 'unexpected'
