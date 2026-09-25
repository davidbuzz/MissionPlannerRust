# A script for tests/gui/fly-scripts.gui: prints, reads the vehicle's state through `cs`, calls
# the `Script` object, and ends. Test scaffolding, not one of Mission Planner's scripts.
print('hello from the Scripts tab')
print('lat', cs.lat)
print('param', Script.GetParam('RC3_MIN'))
print('mode', Script.ChangeMode('Stabilize'))
Script.Sleep(200)
print('done')
