# Where to put the 6 ESP32 nodes

RuView's nodes sense people through WiFi Channel State Information: a body standing in or near the
straight line between two radios changes that link. Six nodes give **15 node-to-node links**, so the
goal is to make those links criss-cross the space where people are, from as many directions as
possible, at body height. `esp_place.py` turns this into numbers for your scanned room; the rules below
are what it optimises and what to check by eye.

## The short version

1. **Spread them around the room's walls**, roughly evenly, ideally 1-2 per wall. Never all on one
   wall, and no two closer than about **1.5 m** (they would give almost identical links).
2. **Mix two heights**: about half at **0.8-1.0 m** (desk / outlet level) and half at **1.8-2.0 m**
   (top of a shelf or wardrobe). Links between a low and a high node slope through the room and see
   both sitting and standing people.
3. **Opposite corners and opposite walls matter most.** The links that cross the middle of the room
   are the ones that carry the signal. Put the area you care most about (bed, sofa, desk, hallway)
   between nodes, not next to one.
4. **Keep 0.3 m of free space around each antenna**, and stand the board upright with the antenna end
   up or sideways and nothing metal in front of it.
5. **Keep them in sight of each other.** Through one interior wall is fine; through a fridge, a mirror,
   a metal shelf or a radiator is not.

## Things to avoid

| Avoid | Why |
|---|---|
| Behind or on a TV, metal shelf, fridge, radiator, mirror | metal reflects and blocks 2.4 GHz; the link sees the metal, not the room |
| Inside cupboards or closed enclosures | blocks the signal and the boards run hot (the firmware keeps the radio always on) |
| On the floor, or under a bed or desk | furniture and legs block the links; the presence detector sees people's feet only |
| Next to a fan, air purifier, washing machine or microwave | moving parts and microwave ovens look like motion: the firmware README warns of false presence |
| Directly next to the WiFi router (< 1 m) | the router's own traffic swamps the node; 2-4 m from it is better |
| Two nodes next to each other | the two links they make are nearly identical; you lose geometry |
| Coin-sized boards (S3 Zero, SuperMini) stacked or enclosed | the firmware README reports overheating; give them air and check by touch in the first minutes |
| A 5 GHz-only network | ESP32-S3 is 2.4 GHz only; the nodes must join a 2.4 GHz SSID |

## Power and network

* Each node needs USB 5 V (a phone charger is fine, about 0.5 W each). Use short, good cables; a
  power-only cable is fine once the board is flashed.
* All nodes join the **same 2.4 GHz network** and stream UDP to the computer running the RuView
  sensing server (port 5005). Give that computer a fixed IP or a DHCP reservation, because the nodes
  have its IP stored.
* Signal: `esp_fleet.py monitor` shows each node's RSSI. Better than -70 dBm is good; if a node is
  weaker, move it or the router, not the other nodes.

## After placing them

1. Power them up in an **empty room** and leave the room for a minute: the presence detector learns
   the empty room for 60 s after boot (firmware README, "Tier 2"). Do this again whenever you move a
   node.
2. Run `python esp_fleet.py monitor`: every node should show 20+ CSI packets per second.
3. **Scan the room with the phone app**, with the node tags (`esp_tags.py`) stuck on the boards, and
   film each tag for a second from about 0.5-1.5 m. The app (or `esp_locate.py auto`) puts every
   node into the 3D scan. Or tap each board in the app, live or later in a video frame.
4. Check the placement against the scan: `python esp_place.py --session <session> --check <session>/esp_nodes.json`
   prints how much of the floor is crossed by 2+ links and from 3+ directions, and flags nodes that are
   too low or too close together.

## What the planner computes

`esp_place.py` builds a 10 cm floor grid from the scan (mesh, or fused depth), finds the walls, and
tries wall positions at 0.9 m and 1.9 m. For each floor cell it samples body points at 0.5, 1.0 and
1.5 m and counts a link as "seeing" the cell if the detour from one node via that point to the other
is at most 0.20 m longer than the direct path (about the first two Fresnel zones at 2.4 GHz, roughly
+-0.6 m around the middle of a 4 m link). The score rewards cells crossed by up to 4 links and from up
to 3 different directions, and penalises nodes closer than 1.5 m and all-one-height layouts. A greedy
pick plus swap search gives the plan; `esp_plan.png` shows coverage, nodes (red = high, orange = low)
and links.

This is a geometry heuristic, not a radio simulation: it can't see walls' materials or furniture made of
metal, so use the table above on top of it. RuView's own docs say location and multi-person accuracy
still need room-specific calibration.
