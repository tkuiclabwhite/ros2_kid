馬達控制 (Motion Control)
-------------------------------
.. currentmodule:: API

.. automethod:: API.sendBodySector
.. automethod:: API.sendHeadMotor
.. automethod:: API.sendSingleMotor

.. tabs::

   .. tab:: 小人型 (KID)

      body 馬達 ID 範圍：**1~21**

      頭部馬達（ID 22 水平、ID 23 垂直）請使用 ``sendHeadMotor``\ 。

   .. tab:: 大人型 (ADULT)

      body 馬達 ID 範圍：**1~27**

      頭部馬達（ID 28 水平、ID 29 垂直）請使用 ``sendHeadMotor``\ 。

.. automethod:: API.SingleAbsolutePosition