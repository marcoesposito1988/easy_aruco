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
@click.option('--squares_x', prompt=True, type=click.INT, help='The number of squares of the board in the X direction')
@click.option('--squares_y', prompt=True, type=click.INT, help='The number of squares of the board in the Y direction')
@click.option('--square_size', prompt='Square size (mm)', type=click.FLOAT, help='The size of each square in millimeters')
@click.option('--marker_size', prompt='Marker size (mm)', type=click.FLOAT, help='The size of each marker embedded in the white squares in millimeters')
@click.option('--output_path', prompt=True, type=click.Path(writable=True), help='The path in which the PDF with the marker should be saved')
def generate_marker(dictionary, squares_x, squares_y, square_size, marker_size, output_path):
    """Script for the generation of ChArUco marker boards"""
    square_size_m = square_size / 1000
    marker_size_m = marker_size / 1000

    complete_path = os.path.abspath(os.path.expanduser(output_path))

    A4_SIZE_m = (0.297, 0.21)
    PAGE_RESOLUTION = (3508, 2480)
    PAGE_PIXELS_PER_METER = np.array(PAGE_RESOLUTION) / np.array(A4_SIZE_m)
    image_resolution = np.around(
        PAGE_PIXELS_PER_METER * np.array((squares_x, squares_y)) * np.array((square_size_m, square_size_m))).astype(
        'int')

    if squares_y * square_size_m > A4_SIZE_m[0]:
        raise ValueError('given height exceeds A4')
    if squares_x * square_size_m > A4_SIZE_m[1]:
        raise ValueError('given width exceeds A4')
    if marker_size > square_size:
        raise ValueError('the size of the marker must be less than the size of the chessboard squares')

    aruco_dict = cv2.aruco.Dictionary_get(getattr(cv2.aruco, dictionary))
    board = cv2.aruco.CharucoBoard_create(squares_x, squares_y, square_size_m, marker_size_m, aruco_dict)
    imboard = board.draw((image_resolution[0], image_resolution[1]))

    f = tempfile.NamedTemporaryFile(suffix='.png', delete=False)

    cv2.imwrite(f.name, imboard)

    board_size_x_mm = squares_x * square_size_m * 1000
    board_size_y_mm = squares_y * square_size_m * 1000

    pdf = FPDF('P', 'mm', 'A4')

    pdf.add_page('P')

    pdf.image(f.name, x=(A4_SIZE_m[1] / 2 * 1000) - board_size_x_mm / 2,
              y=(A4_SIZE_m[0] / 2 * 1000) - board_size_y_mm / 2, w=board_size_x_mm, h=board_size_y_mm)

    print('Configuration:')
    print('Marker size in millimeters: ' + str(marker_size_m * 1000))
    print('Square size in millimeters: ' + str(square_size_m * 1000))
    print('Board squares: ' + str((squares_x, squares_y)))
    print('Total board size in millimeters: ' + str((board_size_x_mm, board_size_y_mm)))
    print('ArUco Dictionary name: ' + dictionary)

    print('Saving the marker as an A4 sized PDF in ' + complete_path)

    pdf.output(complete_path)

    print('REMEMBER TO TURN OFF THE AUTOMATIC RESCALING OF THE PRINTER!')


if __name__ == '__main__':
    generate_marker()
