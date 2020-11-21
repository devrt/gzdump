Utility command to dump/undump pose of objects in gazebo

Install
-------

```
pip install git+https://github.com/devrt/gzdump
```

Usage
-----

```
gzdump [--base-frame world]
```

Dump all the objects in the simulation world in [name, x, y, z, r, p, y] format (in meters, degrees).

```
gzundump [--base-frame world] [--offset-x 0] [--offset-y 0] [--offset-z 0]
```

Undump objects pose from stdin in [name, x, y, z, r, p, y] format (in meters, degrees).

Example
-------

```
gzdump > poses.txt
```

Save current poses to the file.

```
gzdump | gzundump --offset-z -1
```

Add one meters offset along with Z axes to all the objects in the world.

License
-------

MIT

Author
------

Yosuke Matsusaka