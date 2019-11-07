#!/usr/bin/env python

import serial
import time
import threading


class IDMindSerial:
    """Driver class to be inherited"""

    call_number = 0
    fail_number = 0
    lock = threading.Lock()

    def __init__(self, addr, baudrate=115200,timeout=2):
        """Initiates the connection"""
        try:
            self.ser = serial.Serial(addr, baudrate=baudrate, timeout=timeout)
            print "Connection to "+addr+" was successful"
        except serial.SerialException as e:
            print("Connection to "+addr+" failed with: " + str(e))
            self.ser_flag = False
            raise serial.SerialException(e)

    def to_bytes(self, val):
        val=int(val)
        return [(val >> 8) & 0xFF, val & 0xFF]

    def to_num(self, bval):
        res = (ord(bval[0]) << 8) | (ord(bval[1]) & 0xFF)
        if res > 32767: res = res - 65536
        return res

    def send_command(self, msg, tries=5):
        """Receives a list. Checks if the port is open. If needed, formats the message.
        Tries to send the message multiple times"""
        if not self.ser.is_open:
            return 0
        t = 0
        if type(msg) == int or type(msg) == float: msg = [msg]
        while t < tries:
            try:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                return self.ser.write(bytearray(msg))
            except serial.SerialException as e:
                t=t+1
                time.sleep(0.01)
                if t == tries -1: print "Error while sending message: "+str(e)
        return 0

    def read_command(self, nr_bytes, tries=5):
        """Attempts to read from serial"""
        if not self.ser.is_open:
            return None
        t = 0
        while t < tries:
            try:
                return self.ser.read(nr_bytes)
            except serial.SerialException as e:
                t=t+1
                time.sleep(0.01)
                if t == tries - 1: print "Error while reading message: " + str(e)
        return None

    def is_open(self, tries=5):
        t=0
        while t < tries:
            try:
                return self.ser.is_open
            except:
                t = t +1

    def command(self, send, n_read, read=[]):
        """
        This Function executes a serial write, followed by a serial read. Thread Safe.
        :param send: the byte buffer to be written
        :param n_read: the number of bytes to read
        :param read: the buffer to output read results
        :return: the success of the operation
        """
        self.call_number += 1
        self.lock.acquire()
        if self.send_command(send) != len(send):
            self.lock.release()
            self.fail_number += 1
            return False
        else:
            response_buffer = self.read_command(n_read)
            self.lock.release()
            if len(response_buffer) != n_read:
                return False
            checksum = self.to_num(response_buffer[-2:])
            bytesum = reduce(lambda x, y: x + y, [ord(el) for el in response_buffer[:-2]])
            if ord(response_buffer[0]) == send[0] and checksum == (bytesum & 0xffff):
                read.extend(response_buffer[1:-2])
                return True
            else:
                self.fail_number += 1
                return False

    def __del__(self):
        try:
            if self.ser.is_open:
                self.ser.close()
                return True
            else:
                return False
        except serial.SerialException as e:
            print "Exception while attempting to close port: "+str(e)
