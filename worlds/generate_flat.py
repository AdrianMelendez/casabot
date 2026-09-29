#!/usr/bin/env python3
"""Generate worlds/flat.sdf: a furnished 12 x 9 m flat for mapping tests.

    python3 worlds/generate_flat.py > worlds/flat.sdf

Layout (y up, metres, origin at the bottom-left outside corner):

    y 9 +---------------------------------------------+
        |  living room          |      kitchen        |
        |  sofa, coffee table,  |  counters, island,  |
        |  dining table+chairs  |  stools, fridge     |
    4.7 +----   --------------------------   ---------+
        |                corridor                     |
    3.5 +------  ---+-------  ---+--------  ---------+
        |  bedroom  | bathroom   |      office        |
        |  bed,     | bath, WC,  |  desk, chair,      |
        |  wardrobe | sink       |  shelf, couch      |
    y 0 +-----------+------------+--------------------+
       x 0         4.5           7                   12

The robot's lidar sits ~11 cm off the floor, so furniture is modelled the way
it looks at that height: tables and chairs are their legs (thin cylinders)
plus a top the lidar never sees, stools are a single post, beds and sofas are
solid. Thin legs are what make a real flat hard to map and to drive through.
"""

WALL_T = 0.1
WALL_H = 2.5
LEG_R = 0.02

parts = []  # (name, shape, x, y, z_centre, dims, rgb)


def box(name, x, y, sx, sy, sz, z0=0.0, rgb=(0.55, 0.45, 0.35)):
    parts.append((name, 'box', x, y, z0 + sz / 2, (sx, sy, sz), rgb))


def cyl(name, x, y, r, h, z0=0.0, rgb=(0.3, 0.3, 0.3)):
    parts.append((name, 'cylinder', x, y, z0 + h / 2, (r, h), rgb))


WALL_RGB = (0.85, 0.83, 0.8)


def wall(name, a0, a1, at, horizontal, gaps=()):
    """A straight wall from a0 to a1 along x (horizontal) or y, with door gaps."""
    cuts = sorted(gaps)
    edges = [a0] + [v for g in cuts for v in g] + [a1]
    for i in range(0, len(edges), 2):
        s, e = edges[i], edges[i + 1]
        if e - s < 1e-6:
            continue
        mid, length = (s + e) / 2, e - s
        if horizontal:
            box(f'{name}_{i // 2}', mid, at, length, WALL_T, WALL_H, rgb=WALL_RGB)
        else:
            box(f'{name}_{i // 2}', at, mid, WALL_T, length, WALL_H, rgb=WALL_RGB)


def table(name, x, y, sx, sy, h=0.75):
    box(f'{name}_top', x, y, sx, sy, 0.03, z0=h - 0.03)
    dx, dy = sx / 2 - 0.06, sy / 2 - 0.06
    for i, (px, py) in enumerate(((dx, dy), (-dx, dy), (dx, -dy), (-dx, -dy))):
        cyl(f'{name}_leg{i}', x + px, y + py, LEG_R, h - 0.03)


def chair(name, x, y, back):
    """Four legs, a seat at 0.45 m and a backrest on side back = N/S/E/W."""
    s = 0.42
    box(f'{name}_seat', x, y, s, s, 0.03, z0=0.42, rgb=(0.35, 0.25, 0.2))
    d = s / 2 - 0.03
    for i, (px, py) in enumerate(((d, d), (-d, d), (d, -d), (-d, -d))):
        cyl(f'{name}_leg{i}', x + px, y + py, 0.015, 0.42)
    ox, oy = {'N': (0, d), 'S': (0, -d), 'E': (d, 0), 'W': (-d, 0)}[back]
    bx, by = (s, 0.03) if back in 'NS' else (0.03, s)
    box(f'{name}_back', x + ox, y + oy, bx, by, 0.45, z0=0.45, rgb=(0.35, 0.25, 0.2))


def stool(name, x, y):
    cyl(f'{name}_post', x, y, 0.03, 0.65)
    cyl(f'{name}_seat', x, y, 0.18, 0.04, z0=0.65, rgb=(0.2, 0.2, 0.2))


# --- walls ------------------------------------------------------------------
wall('outer_s', 0, 12, 0, True)
wall('outer_n', 0, 12, 9, True)
wall('outer_w', 0, 9, 0, False)
wall('outer_e', 0, 9, 12, False)
wall('corridor_s', 0, 12, 3.5, True, gaps=[(2.8, 3.7), (5.3, 6.1), (9.5, 10.4)])
wall('corridor_n', 0, 12, 4.7, True, gaps=[(1.3, 2.9), (9.0, 10.0)])
wall('bed_bath', 0, 3.5, 4.5, False)
wall('bath_office', 0, 3.5, 7.0, False)
wall('living_kitchen', 4.7, 7.0, 7.5, False)

# --- living room ------------------------------------------------------------
box('sofa_long', 1.6, 8.45, 2.2, 0.9, 0.45, rgb=(0.3, 0.35, 0.5))
box('sofa_long_back', 1.6, 8.85, 2.2, 0.2, 0.4, z0=0.45, rgb=(0.3, 0.35, 0.5))
box('sofa_side', 0.5, 7.2, 0.9, 1.6, 0.45, rgb=(0.3, 0.35, 0.5))
table('coffee_table', 2.4, 7.1, 1.0, 0.6, h=0.45)
box('armchair', 4.3, 7.5, 0.8, 0.8, 0.45, rgb=(0.3, 0.35, 0.5))
box('tv_stand', 5.0, 4.97, 1.6, 0.4, 0.5)
box('bookshelf', 0.22, 5.6, 0.3, 1.2, 1.8)
table('dining_table', 5.8, 7.8, 1.6, 0.9)
chair('dining_chair_n1', 5.4, 8.55, 'N')
chair('dining_chair_n2', 6.2, 8.55, 'N')
chair('dining_chair_s1', 5.4, 7.05, 'S')
chair('dining_chair_s2', 6.2, 7.05, 'S')
cyl('plant_living', 7.1, 8.6, 0.2, 0.8, rgb=(0.2, 0.5, 0.2))

# --- kitchen ----------------------------------------------------------------
box('counter_north', 10.0, 8.65, 3.5, 0.6, 0.9, rgb=(0.7, 0.7, 0.72))
box('counter_east', 11.65, 6.9, 0.6, 2.0, 0.9, rgb=(0.7, 0.7, 0.72))
box('island', 9.6, 6.7, 1.4, 0.8, 0.9, rgb=(0.7, 0.7, 0.72))
box('fridge', 11.6, 5.3, 0.7, 0.7, 1.8, rgb=(0.9, 0.9, 0.9))
stool('stool_1', 9.2, 5.95)
stool('stool_2', 10.0, 5.95)

# --- corridor ---------------------------------------------------------------
box('shoe_cabinet', 0.6, 4.47, 1.0, 0.35, 1.0)

# --- bedroom ----------------------------------------------------------------
box('bed', 1.2, 1.2, 1.6, 2.0, 0.5, rgb=(0.8, 0.8, 0.9))
box('nightstand', 2.35, 0.3, 0.45, 0.4, 0.55)
box('wardrobe', 3.8, 0.4, 1.2, 0.6, 2.0)

# --- bathroom ---------------------------------------------------------------
box('bathtub', 5.75, 0.45, 1.7, 0.75, 0.55, rgb=(0.95, 0.95, 0.95))
box('toilet', 6.7, 2.3, 0.4, 0.65, 0.4, rgb=(0.95, 0.95, 0.95))
box('sink_cabinet', 4.95, 2.62, 0.8, 0.45, 0.85)

# --- office -----------------------------------------------------------------
table('desk', 10.8, 0.5, 1.4, 0.7)
cyl('office_chair_post', 10.8, 1.3, 0.03, 0.45)
box('office_chair_seat', 10.8, 1.3, 0.48, 0.48, 0.05, z0=0.45, rgb=(0.15, 0.15, 0.15))
box('office_shelf', 7.25, 1.5, 0.35, 1.5, 1.8)
box('office_couch', 8.4, 3.0, 1.8, 0.8, 0.45, rgb=(0.4, 0.3, 0.3))
cyl('plant_office', 11.6, 3.1, 0.2, 0.8, rgb=(0.2, 0.5, 0.2))


def geometry(shape, dims):
    if shape == 'box':
        return f'<box><size>{dims[0]:.3f} {dims[1]:.3f} {dims[2]:.3f}</size></box>'
    return f'<cylinder><radius>{dims[0]:.3f}</radius><length>{dims[1]:.3f}</length></cylinder>'


def link(name, shape, x, y, z, dims, rgb):
    g = geometry(shape, dims)
    c = ' '.join(f'{v:.2f}' for v in rgb)
    return (f'      <link name="{name}">\n'
            f'        <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>\n'
            f'        <collision name="c"><geometry>{g}</geometry></collision>\n'
            f'        <visual name="v"><geometry>{g}</geometry>'
            f'<material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse></material></visual>\n'
            f'      </link>')


print(f'''<?xml version="1.0"?>
<!-- GENERATED by worlds/generate_flat.py - edit that, not this.
     A furnished 12 x 9 m flat. Spawn the robot in the corridor at (6.0, 4.1). -->
<sdf version="1.9">
  <world name="flat">
    <!-- 4 ms steps; see house.sdf for why. -->
    <physics name="4ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>6 4.5 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.3 -0.9</direction>
    </light>

    <model name="floor">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry></collision>
        <visual name="v">
          <geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry>
          <material><ambient>0.6 0.55 0.5 1</ambient><diffuse>0.65 0.6 0.55 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="flat">
      <static>true</static>
{chr(10).join(link(*p) for p in parts)}
    </model>
  </world>
</sdf>''')
