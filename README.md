#README 

This project contains nodes to receive a parse UWB measurements from a Pozyx tag and from a Decawave EVK 1000 tag.

## Pozyx connector

To launch the node:

```bash
$ roslaunch gtec_rosuwbreader pozyxranging.launch
```

Inside the ```pozyxranging.launch``` file can be selected the USB port where the tag is attached.

## Decawave connector

To launch the node:

```bash
$ roslaunch gtec_rosuwbreader dwranging.launch
```

Inside the ```dwranging.launch``` file can be selected the USB port where the tag is attached