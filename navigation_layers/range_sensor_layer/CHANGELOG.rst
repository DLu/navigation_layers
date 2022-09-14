^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package range_sensor_layer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2018-08-31)
------------------
* Clean up with roscompile
* range sensor layer: Remove parameter max_angle (`#38 <https://github.com/DLu/navigation_layers/issues/38>`_)
  Any value set for max_angle gets overwritten in updateCostmap() so the
  parameter can safely be removed.
* Issue `#22 <https://github.com/DLu/navigation_layers/issues/22>`_: on updateCostmap, only touch cells within the triangular projection of the sensor cone. I also add a parameter to regulate (and deactivate) this feature (`#30 <https://github.com/DLu/navigation_layers/issues/30>`_)
* Add buffer clearing when calling deactivate() or activate(). (`#33 <https://github.com/DLu/navigation_layers/issues/33>`_)
* Contributors: David V. Lu, Jorge Santos Simón, Krit Chaiso, nxdefiant

0.2.4 (2014-12-10)
------------------

0.2.3 (2014-12-10)
------------------
