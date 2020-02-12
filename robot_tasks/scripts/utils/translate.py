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

def int_from_color(color):
    if color == 'R':
        return msg.ObjectDetection.COLOR_RED
    elif color == 'G':
        return msg.ObjectDetection.COLOR_GREEN
    elif color == 'B':
        return msg.ObjectDetection.COLOR_BLUE
    elif color == 'O':
        return msg.ObjectDetection.COLOR_ORANGE
    else:
        return msg.ObjectDetection.COLOR_UNKNOWN
