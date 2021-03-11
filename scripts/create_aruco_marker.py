#!/usr/bin/env python

import tempfile
from fpdf import FPDF
import cv2
import os
import numpy as np
import click


ARUCO_DICTIONARIES = (
    'DICT_4X4_50',
    'DICT_4X4_100',
    'DICT_4X4_250',
    'DICT_4X4_1000',

    'DICT_5X5_50',
    'DICT_5X5_100',
    'DICT_5X5_250',
    'DICT_5X5_1000',

    'DICT_6X6_50',
    'DICT_6X6_100',
    'DICT_6X6_250',
    'DICT_6X6_1000',

    'DICT_ARUCO_ORIGINAL',
)


@click.command()
@click.option('--dictionary', prompt=True, type=click.Choice(ARUCO_DICTIONARIES), help='ArUco dictionary to be used to create the board')
@click.option('--marker_id', prompt='Marker ID', type=click.INT, help='The marker ID')
@click.option('--marker_size', prompt='Marker size (mm)', type=click.FLOAT, help='The size of the marker')
@click.option('--output_path', prompt=True, type=click.Path(writable=True), help='The path in which the PDF with the marker should be saved')
def generate_marker(dictionary, marker_id, marker_size, output_path):
    marker_size_m = marker_size / 1000

    complete_path = os.path.abspath(os.path.expanduser(output_path))

    A4_SIZE_m = (0.297, 0.21)
    PAGE_RESOLUTION = (3508, 2480)
    PAGE_PIXELS_PER_METER = np.array(PAGE_RESOLUTION)/np.array(A4_SIZE_m)
    marker_size_pixels = np.around(PAGE_PIXELS_PER_METER * marker_size_m).astype('int')

    if marker_size_m > A4_SIZE_m[0] or marker_size_m > A4_SIZE_m[1]:
        raise ValueError('given size exceeds A4')

    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    imboard = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size_pixels[0])

    f = tempfile.NamedTemporaryFile(suffix='.png', delete=False)

    cv2.imwrite(f.name, imboard)

    marker_size_x_mm = marker_size_m * 1000
    marker_size_y_mm = marker_size_m * 1000

    pdf = FPDF('P', 'mm', 'A4')

    pdf.add_page('P')

    pdf.image(f.name, x=(A4_SIZE_m[1]/2*1000)-marker_size_x_mm/2, y=(A4_SIZE_m[0]/2*1000)-marker_size_y_mm/2, w=marker_size_x_mm, h=marker_size_y_mm)

    print('Configuration:')
    print('Marker size in millimeters: '+str(marker_size_x_mm))
    print('Marker ID: ' + str(marker_id))
    print('ArUco Dictionary name: ' + dictionary)

    print('Saving the marker as an A4 sized PDF in '+os.path.abspath(os.path.expanduser(complete_path)))

    pdf.output(complete_path)

    print('REMEMBER TO TURN OFF THE AUTOMATIC RESCALING OF THE PRINTER!')


if __name__ == '__main__':
    generate_marker()
