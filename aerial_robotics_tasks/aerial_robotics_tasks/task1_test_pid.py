#!/usr/bin/env python3

from task1_pid import PID
import time
import plotly.graph_objects as go

pid = PID()
pid.set_kp(0.2)
pid.set_ki(1)
pid.set_kd(0.1)
pid.set_lim_up(1)
pid.set_lim_low(-1)

error = [0,0,0,0.4,0.4,0.4,0.4,0.4,0.5,0.6,0.7,0.8,0.9,0,0,0,0,0,
         -0.4,-0.4,-0.4,-0.4,-0.4,-0.5,-0.6,-0.7,-0.8,-0.9,0,0,0,0,0]

up = []
ui = []
ud = []
u = []
t = []

dt = 0.25
for i, e in enumerate(error):
    print("Error: " + str(e))
    ctl_value = pid.compute(e, 0)
    u_values = pid.get_pid_values()
    up.append(u_values[0])
    ui.append(u_values[1])
    ud.append(u_values[2])
    u.append(u_values[3])
    t.append(i*dt)
    time.sleep(dt)

fig = go.Figure()
fig.add_trace(go.Scatter(x=t, y=error, mode='lines', name='error', line=dict(color='cyan')))
fig.add_trace(go.Scatter(x=t, y=up, mode='lines', name='up', line=dict(color='blue')))
fig.add_trace(go.Scatter(x=t, y=ui, mode='lines', name='ui', line=dict(color='green')))
fig.add_trace(go.Scatter(x=t, y=ud, mode='lines', name='ud', line=dict(color='magenta')))
fig.add_trace(go.Scatter(x=t, y=u, mode='lines', name='u_total', line=dict(color='red')))

fig.update_layout(
    title='PID Controller',
    xaxis_title='Time [s]',
    yaxis_title='Control values',
    legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1)
)

fig.write_html("pid_plot.html")
print("Plot saved as pid_plot.html")
