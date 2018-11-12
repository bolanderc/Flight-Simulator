#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Sep 25 12:33:14 2018

@author: christian
MAE 6510 Flight Simulation
Main simulation code.
"""


import simulation
import aircraft


def flight_simulator(filename, t0):
    a_obj = aircraft.Aircraft()
    dt, t_lim = simulation.load_file(filename, a_obj)
    simulation.initialize(a_obj)
    simulation.run_sim(a_obj, t0, dt, t_lim)


flight_simulator('11.24_input.json', 0.0)
