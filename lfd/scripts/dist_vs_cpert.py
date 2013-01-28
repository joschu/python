#!/usr/bin/env python

import curve_perturbation as cpert
import registration
import numpy as np
import tps
import recognition

import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--s_min', action='store', type=float, default=0.001, help='variance of randomness to introduce to the b-spline control points')
parser.add_argument('--s_max', action='store', type=float, default=0.01)
parser.add_argument('--s_steps', action='store', type=int, default=10)
parser.add_argument('--n', action='store', type=int, default=10, help='num samples to draw')
parser.add_argument('--const_radius', action='store_true', help='don\'t use gaussian around each control point with variance s (just use a random angle, with constant radius sqrt(s)')
parser.add_argument('--method', choices=['tpsrpm', 'meandist', 'geodesic'], default='tpsrpm')
parser.add_argument('input')
args = parser.parse_args()

def d_meandist(curve_nk, pcurve_nk):
    assert curve_nk.shape[0] == pcurve_nk.shape[0]
    return np.sqrt(((curve_nk - pcurve_nk)**2).sum(axis=1)).mean(), None

def d_geodesic(curve_nk, pcurve_nk):
    return recognition.calc_match_score(curve_nk, pcurve_nk), None

def d_tpsrpm(curve, pcurve):
    f, info = registration.tps_rpm(curve, pcurve, return_full=True, verbose=False)
    bend_coef = 0.001
    res_cost, bend_cost, total_cost = tps.tps_err_eval(f.lin_ag, f.trans_g, f.w_ng, info['x_Nd'], info['targ_Nd'], bend_coef, return_tuple=True)
    extra = {
        'bend_cost': bend_cost,
        'res_cost': res_cost
    }
    return total_cost, extra

# def tpsrpm_vs_cpert(curve, s, n):
#     costs = np.zeros((n, 3))
#     for i in range(n):
#         pcurve = cpert.perturb_curve(curve, s, args.const_radius)
#         f, info = registration.tps_rpm(curve, pcurve, return_full=True, verbose=False)

#         bend_coef = 0.001
#         res_cost, bend_cost, total_cost = tps.tps_eval2(f.lin_ag, f.trans_g, f.w_ng, info['x_Nd'], info['targ_Nd'], bend_coef, return_tuple=True)

#        #print "cost = residual + bend"
#        #print " %.3g = %.3g + %.3g" % (total_cost, res_cost, bend_cost)
#         costs[i,:] = total_cost, res_cost, bend_cost

#     print costs.mean(axis=0)
#     return costs

def main():
    print 'using distance function', args.method
    fn = {
        'tpsrpm': d_tpsrpm,
        'meandist': d_meandist,
        'geodesic': d_geodesic,
    }[args.method]

    import matplotlib.pyplot as plt
    curve = np.loadtxt(args.input)
    for s in np.linspace(args.s_min, args.s_max, args.s_steps):
        costs = []
        for _ in range(args.n):
            pcurve = cpert.perturb_curve(curve, s, args.const_radius)
            cost, extra = fn(curve, pcurve)
            costs.append(cost)
        plt.scatter(np.repeat(s, args.n), costs)
        print np.mean(costs)
    plt.show()

if __name__ == '__main__':
    main()
