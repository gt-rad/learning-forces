import os
import numpy as np

import plotly
import plotly.graph_objs as go

import util

def plotplotly(scatterSet, plottitle, xtitle, ytitle, plotfilename, showLegend=True, legendPos=(0.05, 0.95), errorbars=True, boundToPos=False, negate=False, baseline=None, dtick=None):
    data = []
    # Plot background error bars
    maxy = 0
    if errorbars and 'std' in scatterSet[0]:
        for i, s in enumerate(scatterSet):
            xstd = s['x'].tolist() + s['x'][::-1].tolist()
            ystd = (s['y'] + s['std']).tolist() + (s['y'] - s['std'])[::-1].tolist()
            maxy = max(maxy, np.max(ystd))
            data.append(go.Scatter(x=xstd, y=ystd if not negate or i >= len(scatterSet) - 1 else -np.array(ystd), showlegend=False, name=s['name'], fillcolor=s['stdcolor'], line=dict(color=('transparent'))))
    # Plot baseline
    if baseline is not None:
        data.append(go.Scatter(x=scatterSet[0]['x'], y=[baseline]*len(scatterSet[0]['x']), name='Baseline', line=dict(color='rgb(100,60,180)', dash='dash'), mode='lines'))
    # Plot main lines
    for i, s in enumerate(scatterSet):
        maxy = max(maxy, np.max(s['y']))
        data.append(go.Scatter(x=s['x'], y=s['y'] if not negate or i >= len(scatterSet) - 1 else -np.array(s['y']), name=s['name'], line=dict(color=s['color'], dash='solid')))
    layout = dict(title=plottitle,
                  titlefont=dict(size=20),
                  xaxis=dict(title=xtitle, showgrid=True, titlefont=dict(size=18), tickfont=dict(size=18)),
                  yaxis=dict(title=ytitle, showgrid=True, titlefont=dict(size=18), tickfont=dict(size=18), dtick=dtick),
                  # width=1280,
                  # height=720,
                  width=1280 / 1.6,
                  height=720 / 1.8,
                  legend=dict(x=legendPos[0], y=legendPos[1], font=dict(size=18)),
                  showlegend=showLegend)
    if boundToPos:
        layout['yaxis']['range'] = [0, maxy]
    # Make plot directory if necessary
    filedir = os.path.dirname(plotfilename)
    if not os.path.exists(filedir):
        os.makedirs(filedir)
    plotly.offline.plot({'data': data, 'layout': layout}, filename=plotfilename)

def plotComparison(namesearch, directories, title, plottag='', isArm=True, xRotation=False, yRotation=False, zRotation=False):
    filenames = list(set(util.getFilenames(directories, parentDir=util.comparisonDir, namesearch=namesearch)))
    rmse = dict()

    for filename in filenames:
        data = util.loadFile(filename)
        if xRotation or yRotation or zRotation:
            if isArm:
                velocity = data['rotateFist'][-1]
            else:
                velocity = data['rotateArm'][-1]
        else:
            velocity = float(filename[-9:-5])

        predForces = np.array(data['predForces'])
        predActivations = np.array(data['predActivations'])
        yForces = np.array(data['yForces'])
        yActivations = np.array(data['yActivations'])

        predForces[predActivations < 0] = 0
        predForces[predForces < 0] = 0
        yForces[yActivations < 0] = 0

        if velocity not in rmse:
            rmse[velocity] = []
        rmse[velocity].append(np.sqrt(np.mean(np.square(yForces - predForces))))

    print '-'*5, 'Gown Simulation Variation' if isArm else 'Shorts Simulation Variation', '-'*5

    xVel = []
    yRMSE = []
    for vel, msevalues in rmse.iteritems():
        xVel.append(vel)
        yRMSE.append(np.mean(msevalues))
    xVel, yRMSE = (list(t) for t in zip(*sorted(zip(xVel, yRMSE))))
    if xRotation or yRotation or zRotation:
        xVel = np.degrees(xVel)
        print 'RMSE %s rotations:' % ('x' if xRotation else 'y' if yRotation else 'z'), xVel
    else:
        print 'RMSE velocities:', xVel
    print 'RMSE:', yRMSE

    return xVel, yRMSE

def baselinermse(namesearch, directories, isArm=True):
    filenames = list(set(util.getFilenames(directories, parentDir=util.comparisonDir, namesearch=namesearch)))
    rmse = []
    rmseZero = []
    rmseMean = []

    for filename in filenames:
        data = util.loadFile(filename)
        predForces = np.array(data['predForces'])
        predActivations = np.array(data['predActivations'])
        yForces = np.array(data['yForces'])
        yActivations = np.array(data['yActivations'])

        predForces[predActivations < 0] = 0
        predForces[predForces < 0] = 0
        yForces[yActivations < 0] = 0

        rmse.append(np.sqrt(np.mean(np.square(yForces - predForces))))
        rmseZero.append(np.sqrt(np.mean(np.square(yForces))))
        rmseMean.append(np.sqrt(np.mean(np.square(yForces - np.mean(yForces)))))

    print '-'*5, 'Baseline Gown Simulation' if isArm else 'Baseline Shorts Simulation', '-'*5
    print 'RMSE across all %d sequences:' % len(filenames), np.mean(rmse)
    print 'RMSE for estimation of zero:', np.mean(rmseZero)
    print 'RMSE for estimation of mean of sequence:', np.mean(rmseMean)
    print 'RMSE standard deviation:', np.std(rmse)
    return np.mean(rmse)


baselinearm = baselinermse('loc_*', ['maxtotalforce_2016-09-11_18-26-03_ForceTorqueVelocity_rawPressuremap_neoarm'], isArm=True)
baselineleg = baselinermse('loc_*', ['maxtotalforce_2016-09-11_20-23-2016-09-11orceTorqueVelocity_rawPressuremap_neoleg'], isArm=False)
# baselinearm = 0.00672917562233
# baselineleg = 0.048774519547

# xVel, yRMSE = plotComparison('loc_*_velocity_*', ['2017-02-18_19-00-06_ForceTorqueVelocity_rawPressuremap_neoarm_rotations'], 'Velocity Variation in the Gown Simulation', plottag='_avg_mse_arm', isArm=True)
xVel = [1.0, 1.25, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0]
yRMSE = [0.0036726482372982862, 0.0057172398353246218, 0.010252922475809089, 0.012783337142227907, 0.016496314672146861, 0.02116456829851078, 0.023833804400064725, 0.029415039609584501]
scatters = [dict(x=np.array(xVel)*10.0, y=np.array(yRMSE), name='Speed', color='green', stdcolor='rgba(0,255,0,0.2)')]
plotplotly(scatters, 'End Effector Speed Variation in the Gown Task', 'End Effector Speed (cm/s)', 'Root Mean Squared Error (N)', 'rmse%s.html' % 'velocity_arm', showLegend=True, legendPos=(0.05, 0.95), boundToPos=True, baseline=baselinearm)

# xVel, yRMSE = plotComparison('loc_*_velocity_*', ['2017-02-18_19-03-2017-02-18orceTorqueVelocity_rawPressuremap_neoleg_rotation'], 'Velocity Variation in the Shorts Simulation', plottag='_avg_mse_leg', isArm=False)
xVel = [1.5, 1.75, 2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.75, 3.0]
yRMSE = [0.035241809387308448, 0.059358733213136276, 0.087671148078892619, 0.097224029280673974, 0.10953840347186775, 0.12019251357956845, 0.12943295431845747, 0.14577910914993653, 0.16535431733379052, 0.1740409547345404]
scatters = [dict(x=np.array(xVel)*10.0, y=np.array(yRMSE), name='Speed', color='green', stdcolor='rgba(0,255,0,0.2)')]
plotplotly(scatters, 'End Effector Speed Variation in the Shorts Task', 'End Effector Speed (cm/s)', 'Root Mean Squared Error (N)', 'rmse%s.html' % 'velocity_leg', showLegend=True, legendPos=(0.05, 0.95), boundToPos=True, baseline=baselineleg)

# yRot, yRMSE = plotComparison('loc_*_rotateY_*', ['2017-02-18_19-00-06_ForceTorqueVelocity_rawPressuremap_neoarm_rotations'], 'MSE for variation with the Gown Simulation (N=128)', plottag='_avg_mse_arm', isArm=True, yRotation=True)
# zRot, zRMSE = plotComparison('loc_*_rotateZ_*', ['2017-02-18_19-00-06_ForceTorqueVelocity_rawPressuremap_neoarm_rotations'], 'MSE for variation with the Gown Simulation (N=128)', plottag='_avg_mse_arm', isArm=True, zRotation=True)
yRot = [-15, -10,  -5,  -1,   0,   1,   5,  10,  15]
yRMSE = [0.0110754214340064, 0.0096612690323119657, 0.0065266269534188532, 0.0058168756273949407, 0.0057381632259971673, 0.0055175630375277323, 0.0057598261406577589, 0.0056172983338575715, 0.0057766732698739508]
zRot = [-15, -10,  -5,  -1,   0,   1,   5]
zRMSE = [0.0055238625059319023, 0.0055946469408695862, 0.0053014981283034755, 0.0056458963933153587, 0.0056264870106810423, 0.0061844588722932119, 0.0068444643630761748]
scatters = []
scatters.append(dict(x=np.array(yRot), y=np.array(yRMSE), name='Y Rotation', color='green', stdcolor='rgba(0,255,0,0.2)'))
scatters.append(dict(x=np.array(zRot), y=np.array(zRMSE), name='Z Rotation', color='rgb(30,150,240)', stdcolor='rgba(0,255,0,0.2)'))
plotplotly(scatters, 'Limb Rotation Variation in the Gown Task', 'Rotation (deg)', 'Root Mean Squared Error (N)', 'rmse%s.html' % 'rotation_arm', showLegend=True, legendPos=(0.7, 0.98), boundToPos=True, baseline=baselinearm, dtick=0.004)

# xRot, xRMSE = plotComparison('loc_*_rotateX_*', ['2017-02-18_19-03-2017-02-18orceTorqueVelocity_rawPressuremap_neoleg_rotation'], 'MSE for variation with the Shorts Simulation (N=128)', plottag='_avg_mse_leg', isArm=False, xRotation=True)
# yRot, yRMSE = plotComparison('loc_*_rotateY_*', ['2017-02-18_19-03-2017-02-18orceTorqueVelocity_rawPressuremap_neoleg_rotation'], 'MSE for variation with the Shorts Simulation (N=128)', plottag='_avg_mse_leg', isArm=False, yRotation=True)
# zRot, zRMSE = plotComparison('loc_*_rotateZ_*', ['2017-02-18_19-03-2017-02-18orceTorqueVelocity_rawPressuremap_neoleg_rotation'], 'MSE for variation with the Shorts Simulation (N=128)', plottag='_avg_mse_leg', isArm=False, zRotation=True)
xRot = [-15, -10,  -5,  -1,   0,   1,   5,  10,  15]
xRMSE = [0.049142059157996, 0.05757682563902175, 0.058849036539579028, 0.057724708166642574, 0.059358733213136276, 0.05708780395461352, 0.049442112412315387, 0.045942110405314075, 0.045415046581168447]
yRot = [-15, -10,  -5,  -1,   0,   1,   5,  10,  15]
yRMSE = [0.048171912417132035, 0.048663869510059014, 0.050710534236837891, 0.057735763405231691, 0.059358733213136276, 0.057310383413256158, 0.060033596818097577, 0.062701209742944325, 0.052887893411165621]
zRot = [-15, -10,  -5,  -1,   0,   1,   5,  10,  15]
zRMSE = [0.047208195099307161, 0.044863658106394623, 0.047708191105284528, 0.055830305890622586, 0.059358733213136269, 0.059241231215762129, 0.065573514337048833, 0.069834070882862739, 0.071318752879296593]
scatters = []
scatters.append(dict(x=np.array(xRot), y=np.array(xRMSE), name='X Rotation', color='orange', stdcolor='rgba(0,255,0,0.2)'))
scatters.append(dict(x=np.array(yRot), y=np.array(yRMSE), name='Y Rotation', color='green', stdcolor='rgba(0,255,0,0.2)'))
scatters.append(dict(x=np.array(zRot), y=np.array(zRMSE), name='Z Rotation', color='rgb(30,150,240)', stdcolor='rgba(0,255,0,0.2)'))
plotplotly(scatters, 'Limb Rotation Variation in the Shorts Task', 'Rotation (deg)', 'Root Mean Squared Error (N)', 'rmse%s.html' % 'rotation_leg', showLegend=True, legendPos=(0.7, 0.02), boundToPos=True, baseline=baselineleg)

