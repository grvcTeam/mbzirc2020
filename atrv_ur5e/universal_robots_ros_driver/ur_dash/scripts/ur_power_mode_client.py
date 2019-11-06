#! /usr/bin/env python

import socket
import time
from threading import Lock
from ur_dash.srv import *
import rospy

PORT = 29999              #DEPRECATED BECAUSE OF POSSIBLE UNDEFINED BEHAVIOUR
ADRESS = '192.168.1.249'   #DEPRECATED BECAUSE OF POSSIBLE UNDEFINED BEHAVIOUR
TIMEOUT = 2
lock = Lock()

powerSrv = None
programSrv = None
installationSrv = None
modeSrv = None


def open_connection():
    # rospy.loginfo('open_connection()')
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((rospy.get_param('ur_robot_ip'),
               rospy.get_param('ur_robot_port')),)
    return s


def getResponse(server, buffer_size=44, timeout=TIMEOUT):
    # rospy.loginfo('getResponse(buffer_size=44)')
    response = ""
    server.settimeout(timeout)
    start_time = time.time()
    curr_time = time.time()
    while not response and ((curr_time - start_time) < 5):
        curr_time = time.time()
        try:
            response = server.recv(buffer_size)
            # rospy.loginfo("getResponse[29]{response}: " + response)

        except Exception as e:
            rospy.logerr("Exception thrown while receiving bytes")
            rospy.logerr(e)
            break

    try:
        server.settimeout(0.05)
        flush = server.recv(1024)
        #rospy.loginfo('flush: ' + flush)
    except Exception:
        pass
    finally:
        server.settimeout(TIMEOUT)

    return response.replace('\n', '').replace('\r', '').lstrip().rstrip()


def service_handler_on_off(req):
    try:
        lock.acquire()
        server = open_connection()
        text = getResponse(server, 44)
        # rospy.loginfo('52: '+text)

        resp = UrPowerOnOffResponse()

        if not 'Connected: Universal Robots Dashboard Server' in text:
            resp.actualState = None
            resp.msg = 'Connection was not correct/possible'
            # rospy.loginfo('62:')
        else:
            while(True):
                # rospy.loginfo('65: ')
                server.send('robotmode\r\n')
                mode = getResponse(server, 20)

                rospy.loginfo('69: mode = '+mode)

                if req.requestedState == True:
                    if 'Robotmode: POWER_OFF' in mode:
                        server.send('power on\r\n')
                        rospy.loginfo('70: '+getResponse(server, 20))
                        rospy.sleep(1)
                        continue
                    if 'Robotmode: BOOTING' in mode:
                        rospy.loginfo('74: '+mode)
                        rospy.sleep(1)
                        continue
                    if 'Robotmode: IDLE' in mode:
                        server.send('brake release\r\n')
                        rospy.loginfo('79: '+getResponse(server, 15))
                        rospy.sleep(1)
                        continue
                    if 'Robotmode: POWER_ON' in mode:
                        rospy.sleep(1)
                        continue
                    if 'Robotmode: RUNNING' in mode:
                        resp.actualState = True
                        resp.msg = mode
                        break
                    rospy.logerr('Wrong robot mode: ' + mode)
                    break
                else:
                    if 'Robotmode: RUNNING' in mode:
                        server.send('power off\r\n')
                        rospy.sleep(1)
                        rospy.loginfo(getResponse(server, 12))
                        continue
                    if 'Robotmode: IDLE' in mode:
                        server.send('power off\r\n')
                        rospy.sleep(1)
                        rospy.loginfo('100: '+getResponse(server, 12))
                        continue
                    if 'Robotmode: POWER_ON' in mode:
                        server.send('power off\r\n')
                        rospy.sleep(1)
                        rospy.loginfo('110: '+getResponse(server, 12))
                        continue
                    if 'Robotmode: POWER_OFF' in mode:
                        resp.actualState = False
                        resp.msg = mode
                        break
                    rospy.logerr('Wrong robot state: ' + mode)
                    break

    except Exception as ex:
        rospy.loginfo('108: except = ' + str(ex))
        pass
    finally:
        # server.shutdown(socket.SHUT_RDWR) #could not be garbage collected
        server.close()
        lock.release()
        return resp


def service_handler_mode(req):
    try:
        lock.acquire()
        server = open_connection()
        text = getResponse(server, 44)

        resp = UrRobotModeResponse()

        if not 'Connected: Universal Robots Dashboard Server' in text:
            resp.loadedProgram = None
            resp.programState = None
            resp.robotMode = None
            resp.safetyMode = None
        else:
            server.send('robotmode\r\n')
            resp.robotMode = getResponse(
                server, 20, 0.2).replace('Robotmode: ', '')
            rospy.sleep(0.01)
            server.send('get loaded program\r\n')
            resp.loadedProgram = getResponse(
                server, 60, 0.2).replace('Loaded program: ', '')
            rospy.sleep(0.01)
            server.send('programState\r\n')
            resp.programState = getResponse(server, 7, 0.2)
            rospy.sleep(0.01)
            server.send('safetymode\r\n')
            resp.safetyMode = getResponse(
                server, 33, 0.2).replace('Safetymode: ', '')

    except Exception as ex:
        rospy.logerr('154: except = ' + str(ex))
        pass
    finally:
        # server.shutdown(socket.SHUT_RDWR) #could not be garbage collected
        server.close()
        lock.release()
        return resp


def service_handler_program(req):
    try:
        lock.acquire()
        server = open_connection()
        text = getResponse(server, 44)

        resp = UrRunProgramResponse()

        if not 'Connected: Universal Robots Dashboard Server' in text:
            resp.loadedProgramName = None
            resp.isRunning = None
        else:
            if '.urp' in req.programName:
                server.send('stop\r\n')
                rospy.loginfo('Stopping program on UR')
                getResponse(server, 7, 0.5)
                rospy.sleep(0.5)
                server.send('load ' + req.programName + '\r\n')
                rospy.sleep(1)
                rospy.loginfo(getResponse(server, 120, 1))
                rospy.sleep(1)
            if req.runMode:
                server.send('programState\r\n')
                text = getResponse(server, 40, 0.5)
                if '.urp' in text:
                    rospy.wait_for_service('ur_power_dash')
                    rospy.loginfo('ur_power_dash is up')
                    try:
                        power = rospy.ServiceProxy(
                            'ur_power_dash', UrPowerOnOff)
                        rospy.loginfo('Handle created, invoking')
                        server.close()
                        lock.release()
                        respPower = power(1)
                        lock.acquire()
                        server = open_connection()
                        getResponse(server, 44)
                        rospy.loginfo('ur_power_dash returned: %s' %
                                      respPower.actualState)
                        if respPower.actualState:
                            server.send('play\r\n')
                            getResponse(server, 1, 0.1)
                    except rospy.ServiceException, e:
                        rospy.logfatal("Service call failed: %s" % e)
            else:
                server.send('stop\r\n')
                getResponse(server, 1, 0.1)
            rospy.loginfo('Gathering data at the end.')
            server.send('get loaded program\r\n')
            resp.loadedProgramName = getResponse(
                server, 80, 0.5).replace('Loaded program: ', '')
            rospy.sleep(0.1)
            server.send('running\r\n')
            if 'true' in getResponse(server, 21).lower():
                resp.isRunning = True
            else:
                resp.isRunning = False

    except Exception as ex:
        rospy.logerr('177: except = ' + str(ex))
    finally:
        # server.shutdown(socket.SHUT_RDWR) #could not be garbage collected
        server.close()
        lock.release()
        rospy.loginfo(str(resp))
        return resp


def service_handler_installation(req):
    try:
        lock.acquire()
        server = open_connection()
        text = getResponse(server, 44)

        resp = UrLoadInstallationResponse()

        if not 'Connected: Universal Robots Dashboard Server' in text:
            resp.success = False
        else:
            resp.success = False
            rospy.logerr(
                'UrLoadInstallation is not implemented, do not use it!')
    except Exception as ex:
        rospy.loginfo('199: except = ' + str(ex))
        pass
    finally:
        # server.shutdown(socket.SHUT_RDWR) #could not be garbage collected
        server.close()
        lock.release()
        return resp


def service_init():
    global powerSrv
    global programSrv
    global installationSrv
    global modeSrv
    rospy.init_node('ur_dash_client')
    powerSrv = rospy.Service(
        'ur_power_dash', UrPowerOnOff, service_handler_on_off)
    programSrv = rospy.Service(
        'ur_program_dash', UrRunProgram, service_handler_program)
    # installationSrv = rospy.Service('ur_installation_dash', UrLoadInstallation, service_handler_installation)
    modeSrv = rospy.Service('ur_state_dash', UrRobotMode, service_handler_mode)


if __name__ == "__main__":
    try:
        service_init()
        if not rospy.has_param('ur_robot_ip'):
            rospy.set_param('ur_robot_ip',ADRESS) #DEPRECATED BECAUSE OF POSSIBLE UNDEFINED BEHAVIOUR
        while not rospy.has_param('ur_robot_ip'):
            rospy.logerr_throttle(
                5, "ur_robot_ip parameter is not present on parameter server")
        if not rospy.has_param('ur_robot_port'):
            rospy.set_param('ur_robot_port',PORT) #DEPRECATED BECAUSE OF POSSIBLE UNDEFINED BEHAVIOUR
        while not rospy.has_param('ur_robot_port'):
            rospy.logerr_throttle(
                5, "ur_robot_port parameter is not present on parameter server")
        rospy.loginfo('Init has ended!')
        rospy.spin()

    except rospy.ROSInterruptException as ex:
        rospy.loginfo(ex)
        raise ex

    except rospy.ROSInternalException as ex:
        rospy.loginfo(ex)
        raise ex

    finally:
        powerSrv.shutdown()
        programSrv.shutdown()
        # installationSrv.shutdown()
        modeSrv.shutdown()
        quit()
