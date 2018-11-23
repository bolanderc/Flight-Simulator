# -*- coding: utf-8 -*-
"""
Created on Wed Aug  8 13:09:24 2018

@author: Christian
"""

import numpy as np
import scipy.optimize as optimize
import matplotlib.pyplot as plt


def sinesummationfit(x, y, num_sine=0, maxiter=0, tolerance=0):
    x_down, y_down = down_sample(x, y, num_sine)
    fit_error = np.copy(y_down)
    harmonic_params = np.zeros(num_sine*3 + 1)
    optimized_params = np.zeros(num_sine*3 + 1)
    k = 4
    for i in range(num_sine):
        if i == 0:
            harmonic_params[:4] = signal_decomposition(x_down, fit_error,
                                                       include_offset=True)
            optimized_params[:4] = optimize.minimize(rmse, harmonic_params[:4],
                                                     args=(x_down, y_down, k),
                                                     method='BFGS',
                                                     options={'maxiter':
                                                              maxiter},
                                                     tol=tolerance).x
        else:
            harmonic_params[k:k+3] = signal_decomposition(x_down, fit_error)
            optimized_params[:k+3] = optimize.minimize(rmse,
                                                       harmonic_params[:k+3],
                                                       args=(x_down, y_down,
                                                             k),
                                                       method='BFGS',
                                                       options={'maxiter':
                                                                maxiter},
                                                       tol=tolerance).x
            k += 3
        fit_error = calc_fit_error(optimized_params, x, y, k)
        harmonic_params[:] = optimized_params[:]
#    display_final_fit(harmonic_params, x, y, num_sine)
    return harmonic_params


def signal_decomposition(x, y, include_offset=False):
    N = len(y)
    dt = (x[-1] - x[0])/N
    num_freqs = N//2
    fourier_transform = np.fft.fft(y)
    fourier_one = fourier_transform[:num_freqs]
    fourier_one[1:num_freqs] = 2*fourier_transform[1:num_freqs]
    amp_spec_one = np.absolute(fourier_one)/N
    phase_spec_one = np.pi/2.0 + np.arctan2(fourier_one.imag,
                                            fourier_one.real)
    amplitude = max(amp_spec_one).real
    freq_spec = np.fft.fftfreq(N)/dt
    freq_spec_one = freq_spec[:num_freqs]*2.0*np.pi
    power_spec = (np.abs(fourier_one)**2)*((dt)**2)
    highest_power_index = power_spec.argmax()
    frequency = freq_spec_one[highest_power_index]
    phase = phase_spec_one[highest_power_index]
    if include_offset is True:
        offset = np.average(y)
        harmonic_params = np.array([amplitude, frequency, phase, offset])
    else:
        harmonic_params = np.array([amplitude, frequency, phase])
    return harmonic_params


def down_sample(x, y, num_sines):
    min_period = _find_min_period(x, y, num_sines)
    num_min_periods = (x[-1] - x[0])/min_period
    new_x = np.linspace(x[0], x[-1], num=int(4.0*num_min_periods),
                        endpoint=True)
    new_y = np.interp(new_x, x, y)
    return new_x, new_y


def _find_min_period(x, y, num_sines):
    N = len(y)
    dt = (x[-1] - x[0])/N
    num_freqs = N//2
    fourier_transform = np.fft.fft(y)
    fourier_one = fourier_transform[:num_freqs]
    fourier_one[1:num_freqs] = 2*fourier_transform[1:num_freqs]
    power_spec = (np.abs(fourier_one)**2)*((dt)**2)
    freq_spec = np.fft.fftfreq(N)/dt
    freq_spec_one = freq_spec[:num_freqs]*2.0*np.pi
    peak_indices = np.argpartition(power_spec, -num_sines)[-num_sines:]
    max_freq = freq_spec_one[max(peak_indices)]
    min_period = (2.0*np.pi)/max_freq
    return min_period


def rmse(harmonic_params, x, y, num_params):
    N = len(y)
    summation_fit = np.zeros(N)
    deviation = np.zeros(N)
    A_0 = harmonic_params[0]
    w_0 = harmonic_params[1]
    phi_0 = harmonic_params[2]
    z = harmonic_params[3]
    base_sine = A_0*np.sin(w_0*x + phi_0) + z
    if num_params > 4:
        A_i = harmonic_params[4:num_params:3]
        w_i = harmonic_params[5:num_params:3]
        phi_i = harmonic_params[6:num_params:3]
        summation_fit = base_sine + np.sum((A_i[:, None]*np.sin(
                                     w_i[:, None]*x[None, :] +
                                     phi_i[:, None])), axis=0)
    else:
        summation_fit[:] = base_sine[:]
    deviation = ((summation_fit - y)**2)
    root_mean_square_error = np.sqrt(sum(deviation)/N)
    return root_mean_square_error


def import_data(filename, headerlines=0, start=None, stop=None):
    dataset = np.genfromtxt(filename, delimiter=',',
                            skip_header=headerlines)
    if start is None:
        if stop is None:
            x = dataset[:, 0]
            y = dataset[:, 1]
        else:
            x = dataset[:stop, 0]
            y = dataset[:stop, 1]

    if stop is None:
        if start is None:
            x = dataset[:, 0]
            y = dataset[:, 1]
        else:
            x = dataset[start:, 0]
            y = dataset[start:, 1]
    if start is not None and stop is not None:
        x = dataset[start:stop, 0]
        y = dataset[start:stop, 1]
    return x, y


def calc_fit_error(harmonic_params, x, y, num_params):
    N = len(y)
    error = np.zeros(N)
    y_fit = sum_sines(x, harmonic_params, num_params)
    error = (y - y_fit)
    return error


def sum_sines(x, harmonic_params, num_params):
    A_0 = harmonic_params[0]
    w_0 = harmonic_params[1]
    phi_0 = harmonic_params[2]
    z = harmonic_params[3]
    base_sine = A_0*np.sin(w_0*x + phi_0) + z
    if isinstance(x, (list, tuple, np.ndarray)):
        N = len(x)
        y_fit = np.zeros(N)
        if num_params > 4:
            A_i = harmonic_params[4:num_params:3]
            w_i = harmonic_params[5:num_params:3]
            phi_i = harmonic_params[6:num_params:3]
            y_fit = base_sine + np.sum((A_i[:, None]*np.sin(
                                         w_i[:, None]*x[None, :] +
                                         phi_i[:, None])), axis=0)
        else:
            y_fit[:] = base_sine[:]
    else:
        N = 1
        y_fit = 0.0
        base_sine = A_0*np.sin(w_0*x + phi_0) + z
        if num_params > 4:
            A_i = harmonic_params[4:num_params:3]
            w_i = harmonic_params[5:num_params:3]
            phi_i = harmonic_params[6:num_params:3]
            y_fit = base_sine + np.sum((A_i[:, None]*np.sin(
                                         w_i[:, None]*x +
                                         phi_i[:, None])), axis=0)
        else:
            y_fit = base_sine
    return y_fit




def projection(x, y, len_proj, filename, harmonic_params, header_lines=0,
               start=None):
    num_params = len(harmonic_params)
    N = len(x)
    total_length = N + len_proj
    x_act, y_act = import_data(filename, headerlines=header_lines,
                               start=start, stop=total_length)
    y_predict = sum_sines(x_act, harmonic_params, num_params)
    return x_act, y_act, y_predict


def sine_summation_progression(x, y, num_sine=0, maxiter=0, tolerance=0):
    x_down, y_down = down_sample(x, y, num_sine)
    fit_error = np.copy(y_down)
    harmonic_params = np.zeros(num_sine*3 + 1)
    optimized_params = np.zeros(num_sine*3 + 1)
    y_intermediate = np.zeros((num_sine, len(y)))
    k = 4
    for i in range(num_sine):
        if i == 0:
            harmonic_params[:4] = signal_decomposition(x_down, fit_error,
                                                       include_offset=True)
            optimized_params[:4] = optimize.minimize(rmse, harmonic_params[:4],
                                                     args=(x_down, y_down, k),
                                                     method='BFGS',
                                                     options={'maxiter':
                                                              maxiter},
                                                     tol=tolerance).x
        else:
            harmonic_params[k:k+3] = signal_decomposition(x_down, fit_error)
            optimized_params[:k+3] = optimize.minimize(rmse,
                                                       harmonic_params[:k+3],
                                                       args=(x_down, y_down,
                                                             k),
                                                       method='BFGS',
                                                       options={'maxiter':
                                                                maxiter},
                                                       tol=tolerance).x
            k += 3
        fit_error = calc_fit_error(optimized_params, x, y, k)
        harmonic_params[:] = optimized_params[:]
        y_intermediate[i, :] = sum_sines(x, harmonic_params,
                                         len(harmonic_params))
    return y_intermediate


def display_final_fit(harmonic_params, x, y, num_sines):
    plt.rc('text', usetex=True)
    num_params = num_sines*3 + 1
    A = np.zeros(num_sines)
    w = np.zeros(num_sines)
    phi = np.zeros(num_sines)
#    y_fit = Fsum_sines(x, harmonic_params, num_sines*3+1)
#    plt.figure(0)
#    plt.plot(x, y, label='Original Data')
#    plt.plot(x, y_fit, label='Fit')
    A = harmonic_params[4:num_params:3]
    w = harmonic_params[5:num_params:3]
    phi = harmonic_params[6:num_params:3]
    A = np.insert(A, 0, harmonic_params[0])
    w = np.insert(w, 0, harmonic_params[1])
    phi = np.insert(phi, 0, harmonic_params[2])
    z = harmonic_params[3]
    print("Harmonic Parameters")
    print("-----------------------")
    print("Function:")
    for i in range(num_sines):
        print("%2.4fsin(%2.4ft + %2.4f) + " % (A[i], w[i], phi[i]), end='')
    print("%2.4f" % (z))
    print("----------------------")
    print("Sine\tA\t\t\u03C9 (rad/s)\t\u03C6 (rad)\t\tz")
    for i in range(num_sines):
        if i == 0:
            print("%1d\t%2.8f\t%2.8f\t%2.8f\t%2.8f" % (i+1, A[i], w[i], phi[i],
                                                       z))
        else:
            print("%1d\t%2.8f\t%2.8f\t%2.8f" % (i+1, A[i], w[i], phi[i]))
