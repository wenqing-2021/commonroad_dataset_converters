## How to run

1. Setup of inD data source:
    - Every OS: copy/link recording data (from inD Dataset) in `data` in root of this project
2. Install requirements for this project (Anaconda is recommended)
3. Make sure this project is properly installed (mainly make sure a `.pth` file points to this projects root)
4. Change into the `commonroad_rl/utils_ind` directory
5. If meta scenarios have not been created yet
    - install the [commonroad-map-tool](https://gitlab.lrz.de/cps/commonroad-map-tool) (for support of traffic signs and stop lines, install [feature_osm_convert_trafficsigns](https://gitlab.lrz.de/cps/commonroad-map-tool/-/tree/feature_osm_convert_traffic_signs))
    - convert the osm files
      ```bash
      bash convert_osm.sh
      ``` 
6. Run script (`--help` for options)
   ```bash
   python -m src.utils_ind.ind_to_cr --help
   ```

Typically you would want to run something like the following.
It will convert the recordings 9, 20 and 33 to common road format and output
20 smaller planning problems for each recording, storing them in the directory `../../data_cr`.
```
python -m commonroad_rl.utils_ind.ind_to_cr -t 0 9 20 32 -n 20 -o ../../pickles  --multiprocessing -v -p
```

## Generating gifs with ffmpeg

* run visualize_cr with the `-o` option to fill a directory with animation frames
* cd there
* `ffmpeg -i %03d.png -vf palettegen palette.png` (or however many digits your images have)
* `ffmpeg -i %03d.png -i palette.png -lavfi "paletteuse" -y output.gif`

## Notes on Cartographic data

### Conversion

The underlying cartographic data of the intersections was once converted from
a OpenStreetMap format to a commonroad lanelet network using the [commonroad-map-tool](https://gitlab.lrz.de/cps/commonroad-map-tool/-/tree/develop/).

The tool to be used is osm-convert:

```bash
osm-convert PATH/TO/OSMMAPS/OSMMAP.osm -o LANELETFILE.xml 
```

Since the coordinates there are stored in UTM coordinates (which is considerably too large for commonroad)
they have been edited by subtracting the UTM cooridinates associated with them in the recordingMeta files.
This should can be done by
```bash
python tools/translate_intersections.py inD_LaneletMaps/convert_tinkered
```

### Fixing the provided OSM data

1. remove sidewalks, pedestrian routes and unnecessary walls
2. make sure all paths along one consecutive logical lanelets are pointing in the direction
   -> if not, select path in JOSM, select "reverse path"
3. make sure all lanelets that are supposed to split/join from another lanelet do connect
    to the (same) endpoints of that other lanelet
3. convert to lanelet with [`commonroad-map-tool`](https://gitlab.lrz.de/cps/commonroad-map-tool/-/tree/develop/)'s script `osm-convert`
4. run [`xml-scenario-checker`](https://gitlab.lrz.de/cps/xml-scenario-checker/-/tree/feature_issues_in_network_checker/)'s script `lanelet-network-checker/run_vertices_repair.py`
5. run the logical and geometrical checks of aforementioned package
6. reiterate and fix errors

## Notes on Recordings / Trajectories

### Generation of planning problems

The inD database files store tracks over an immense frame of time.
It is hence to be considered to make use of one recording several times

- Choose one random vehicle as planning problem. Cut time frame to living frames of this vehicle.

This can be done several times within the same recording and can yield non-overlapping planning problems
from one recording only.

### Validation

Since the tracks are recorded using drones, vehicles are bound to "spawn" at the boundary of the scenario.
This is undesired behaviour in the commonroad framework and hence disallowed by the xml specification (initial states time_step is forced to zero).
For this project this does not bother.
A slightly modified .xsd file has thus been created to allow for non-negative time_step values in the initial state of objects.

### Documentation on traffic signs and lights

Since traffic lights mess up vehicle behaviour if the temporal light state is unknown here
is a list of intersections in the dataset featuring traffic lights.
As described in the [inD-paper](https://arxiv.org/abs/1911.07602) all intersections are traffic light free though.

Other traffic signs (such as give way signs) are just as important for correct behaviour prediction so OSM data and aerial data was searched for signs of them (ie. shadows)

| File name | type | position | source | Notes |
| --------- | ------------- | -------- | ------ | ----- |
| Bendplatz.osm | various | | inD dataset | meaning not fully documented in OSM wiki, but in a [ADAC guide on StVO codes](https://erscharter.eu/sites/default/files/resources/traffic_signs_and_signals_in_germany.pdf) [english] or [wikipedia](https://de.wikipedia.org/wiki/Bildtafel_der_Verkehrszeichen_in_der_Bundesrepublik_Deutschland_von_2013_bis_2017) or in the defining [Verkehrszeichenkatalog](http://www.vzkat.de/2017/VzKat.htm) [german] |
| aseag.osm | give way sign | 50.7853132 61308116 | aerial imagery | neuköllner straße (top) entering into charlottenburger alle (bottom) on left side |
| heckstrasse.osm | stop sign | 50.7788996 6.1654376 | OSM | this is in the middle of the road. heckstrase (top right street) entering into von-coels-straße |
| heckstrasse.osm | various | | inD dataset | cf Bendplatz.osm

A complete information on the right of way situation can also be found in the [inD-paper](https://arxiv.org/abs/1911.07602) (Section: the dataset at a glance).
Virtual and actual traffic signs are added to the dataset where possible to be integrated in the commonraod scenarios.

### A note on right of way relations in CommonRoad

TL;DR The format of traffic signs in commonroad wil make determining right of way/yield relations in the gym environment troublesome

Digging into the commonroad scenarios, you will find out that the only way to place "yield to" and "right of way over" relations is - there is none! The only way is to place priority lane and yield signs and link them to the lanelet which they directly affect (i.e. this lanelet has to yield. To whom? Not saved :)).

For once this means wihtout a traffic sign this relation will be completely lost. Also this makes it (as far as I see) highly non-trivial to determine the which lanelets will be on "the opposing side" - i.e. which lanelets will the vehicle have to yield *to* or have the right over. An example on how non-trivial the guesswork on "what lanelets does this traffic sign apply to" can be found in the comments - even inside one (normal?) T-crossing there are several "yield for" relations that would contradict each other if extended too far.

The way I will handle this in the conversion is to place as many virtual traffic signs in the scenarios if possible. Then I suggest interpreting "right of way" as "right of way over everyone until next sign" and conversely for "yield to". If that doesn't suffice we might have to think of other ways (like extending the CR2020 format).

