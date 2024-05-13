"""

    description: Quick Serial Stream Viewer for ESP32CAM's
    usage:       stream_reader.py [-h] -p PORT [-b BAUD]      
    date:        13/05/24
    
    Note:
        - is slow, but convenient for testing ESP32 cam features
        - couldnt find any other examples of this online


serial data of form:
->DELIM
->header containing content-length
->DELIM
->image data
->DELIM    

"""
import numpy as np
import serial
import cv2
import re
import argparse
import sys


# defaults
BAUD_RATE = 115200
MAX_BUF_SIZE =  8192 * 16
DELIM = b'\n12345678900STREAM00987654321\n'


# only raised if image is larger than allocated max buf size
class BufferOverflowError(Exception):
    """ exception to catch overflow errors."""
    pass


def show_image(im_data):
    """ Displays the image """
    im_np = np.frombuffer(im_data, dtype=np.uint8)
    im = cv2.imdecode(im_np, cv2.IMREAD_COLOR)
    if im is not None:
        cv2.imshow('ESP32 STREAM', im)
        cv2.waitKey(1)
    else:
        print('Could not decode image')
    return


def read_serial_data(ser, buf):
    """ Reads data for serial port into buffer """
    try:
        buf += ser.read(ser.in_waiting or 1)
        if len(buf) > MAX_BUF_SIZE:
            print(f'Buffer too big! Max: {MAX_BUF_SIZE}, Current: {len(buf)}')
            buf = buf[-MAX_BUF_SIZE:]
    except serial.SerialException as e:
        print(f'Error reading from serial: {e}')
    return buf


def process_buffer(buf, im_count):
    """ Trys to find and show images in buffer """
    while True:

        # try and find a deliminator in the stream
        delim_idx = buf.find(DELIM)
        if delim_idx == -1:
            # no delim. found, discard buf data so far
            buf = buf[-len(DELIM):]
            break
        
        # try and find the next deliminator
        next_delim_idx = buf.find(DELIM, delim_idx + len(DELIM))
        if next_delim_idx == -1:
            # only first delim. found, more data pls
            break
        
        # extract data packet, see if is header and grab the content length
        header_data = buf[delim_idx + len(DELIM):next_delim_idx]
        im_len = re.search(b'Content-Length: (\d+)', header_data)
        if not im_len:
            # content length not found, packet is not header
            # ignore buf data up to this second delim. so it becomes the 1st one...
            buf = buf[next_delim_idx:]
            continue
        
        im_len = int(im_len.group(1))
        im_start = next_delim_idx + len(DELIM)
        im_end = im_start + im_len

        # checking that there is enough space for the image
        if im_end > MAX_BUF_SIZE:
             raise BufferOverflowError('Buffer not big enough for image!',
                                    f'Max: {MAX_BUF_SIZE}, Required: {im_end}')
        
        # does buf contain the full image yet?!
        if len(buf) >= im_end:
            im_data = buf[im_start:im_end]
            show_image(im_data)
            im_count += 1

            # just for testing really
            print('--'*5,   
                f'\nim num:{im_count}',
                f'\nim len: {im_len}',
                f'\nbuf len: {len(buf)}')
            
            # remove processed data from buf
            buf = buf[im_end + len(DELIM):]
        else:
            #print(f'Incomplete image data. Expected: {im_len}, Available: {len(buf) - im_start}')
            break
    return buf, im_count


def run_viewer(serial_port, baud_rate):
    """ main task with lots of error handling... """
    buf = b''
    im_count = 0
    ser = None
    
    try:
        ser = serial.Serial(serial_port, baud_rate)
        print(f'Connected to {serial_port} at {baud_rate} baud.')
        
        while True:
            buf = read_serial_data(ser, buf)
            buf, im_count = process_buffer(buf, im_count)
            
    except serial.PortNotOpenError as e:
        print(f'Port not open error: {e}')
    except serial.SerialException as e:
        print(f'Error opening serial port {serial_port}: {e}')
    except KeyboardInterrupt:
        print('Keyboard interupt')
    except OSError as e:
        print(f'Serial error: {e}')
    except BufferOverflowError as e:
        print(f'Buffer overflow error: {e}')
    except Exception as e:
        print(f'Unexpected error: {e}')
    finally:
        if ser and ser.is_open:
            ser.close()
        print('Serial port closed')
    return 0


def parse_args():
    """ Parser for args if run from terminal """
    # passing arguments, if run from terminal
    parser = argparse.ArgumentParser(description='Quick ESP32_CAM Serial Stream Viewer')
    parser.add_argument('-p','--port', type=str, required=True,
                        help='Serial port to connect to. \n MacOS: Run ls /dev/tty.* ')
    parser.add_argument('-b', '--baud', type=int, default=BAUD_RATE,
                        help=f'Baud rate for serial communication. Default is {BAUD_RATE}.')
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_args()
    run_viewer(args.port, args.baud)