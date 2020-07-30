#!/bin/bash

mkdir inD_LaneletMaps/convert_tinkered
for LOCATION in aseag Bendplatz frankenberg heckstrasse
do
  osm-convert inD_LaneletMaps/osm_tinkered/$LOCATION.osm -o inD_LaneletMaps/convert_tinkered/$LOCATION.xml -f
done

python tools/translate_intersections.py -i inD_LaneletMaps/convert_tinkered -o inD_LaneletMaps/convert_tinkered_translated
