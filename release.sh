rel=v1.1.0
mkdir release >/dev/null 2>&1
rm -rf release/$rel >/dev/null 2>&1
mkdir release/$rel
cp autopilot-panel/autopilot-panel release/$rel
cp -rp autopilot-panel/settings release/$rel
sudo chown pi:pi release/$rel/settings/*.json
dos2unix release/$rel/settings/*.json
cp release/$rel/settings/default-settings.json release/$rel/settings/autopilot-panel.json
tar -zcvf release/autopilot-panel-$rel-raspi.tar.gz release/$rel
