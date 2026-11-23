.. _power_consumption_sample:

.. ncs-sample::
   :title: Power consumption (System ON Idle)

   The Power consumption (System ON Idle) sample demonstrates how to evaluate the power consumption of the nRF7120 SoC during the System ON Idle state.
   The sample application is set to periodically sleep for a configurable duration, after which the device wakes up from sleep, printing the wake up count.
   In addition to that, the sample demonstrates the different RAM retention levels available through the :ref:`lib_ram_pwrdn` library.

Requirements
************

The sample supports the following development kit:

.. table-from-sample-yaml::

The sample also requires a `Power Profiler Kit II (PPK2)`_, for measuring current consumption.

Overview
********

At boot, the sample performs the following operations:

1. Applies the user-configured RAM retention level.
#. Enters a periodic loop, where it:

   a. Transits to sleep (System ON Idle) for a period of :kconfig:option:`CONFIG_SAMPLE_POWER_CONSUMPTION_IDLE_SECONDS` (10 seconds by default).
   #. Wakes up from sleep.
   #. Prints the wake-up counter value.
   #. Transits to sleep again.

RAM retention levels
=====================

By default, all application RAM is retained during System ON Idle.
The sample's own Kconfig allows the user to configure the amount of RAM that is retained, exposing a choice of retention levels, each backed by the :ref:`lib_ram_pwrdn` library:

.. list-table::
   :header-rows: 1

   * - Kconfig option
     - Behavior
   * - :kconfig:option:`CONFIG_SAMPLE_POWER_CONSUMPTION_RAM_RETAIN_FULL` (default)
     - All RAM stays retained.
       No RAM is powered down.
   * - :kconfig:option:`CONFIG_SAMPLE_POWER_CONSUMPTION_RAM_RETAIN_64K`
     - Only the first 64 KiB of RAM stays retained, the rest is powered down.
   * - :kconfig:option:`CONFIG_SAMPLE_POWER_CONSUMPTION_RAM_RETAIN_128K`
     - Only the first 128 KiB of RAM stays retained, the rest is powered down.
   * - :kconfig:option:`CONFIG_SAMPLE_POWER_CONSUMPTION_RAM_RETAIN_256K`
     - Only the first 256 KiB of RAM stays retained, the rest is powered down.
   * - :kconfig:option:`CONFIG_SAMPLE_POWER_CONSUMPTION_RAM_RETAIN_512K`
     - Only the first 512 KiB of RAM stays retained, the rest is powered down.
   * - :kconfig:option:`CONFIG_SAMPLE_POWER_CONSUMPTION_RAM_RETAIN_USED_ONLY`
     - Retains only the RAM used by the sample image, the rest is powered down using ``power_down_unused_ram()`` instead of a fixed KiB boundary.

Building and running
********************

.. |sample path| replace:: :file:`nrf/samples/benchmarks/power_consumption`

.. include:: /includes/build_and_run.txt

The default configuration (all RAM retained) is built with:

.. code-block:: console

   west build -p always -b nrf7120dk/nrf7120/cpuapp nrf/samples/benchmarks/power_consumption

To build the sample with any other of the supported RAM retention levels, pass the matching Kconfig option on the command line, for example:

.. code-block:: console

   west build -p always -b nrf7120dk/nrf7120/cpuapp nrf/samples/benchmarks/power_consumption -- -DCONFIG_SAMPLE_POWER_CONSUMPTION_RAM_RETAIN_64K=y

The same supported six configurations are also given in the :file:`tests.yaml` file (for example ``sample.benchmarks.power_consumption.ram_retain_64k``), for use with ``west build -T <name>`` or Twister.

Testing
========

|test_sample|

.. _power_consumption_sample_measuring:

Measuring the power consumption
-------------------------------

**P601** is a 1x3 header (pin 1 ``P5V0``, pin 2 ``VBAT_5V0``, pin 3 ``GND``), normally bridged by the **JP601** shunt jumper so ``P5V0`` feeds straight through to ``VBAT_5V0``.
To measure current with a PPK2 in source meter mode, complete the following steps:

1. Remove the **JP601** shunt jumper from **P601**.
#. Connect the PPK2's **Vout** to **P601** pin 2 (``VBAT_5V0``).
#. Connect the PPK2's **GND** to **P601** pin 3 (``GND``).
#. Connect the USB cable to the DK to flash the image (the DK's interface MCU/debugger runs on its own USB-derived supply; with the **JP601** shunt removed, the nRF7120 itself is powered only from **P601** pin 2, i.e. the PPK2, throughout).
#. In the PPK2 app, select **Source Meter** mode and enable power output with **3.6 V** as the supply voltage.
#. Run ``west flash`` to flash the image.

Measuring wake-up latency
-------------------------

Enable :kconfig:option:`CONFIG_SAMPLE_POWER_CONSUMPTION_LATENCY_MEASUREMENT`
to replace ``k_sleep()`` with a dedicated GRTC compare channel and expose three
active-high timing markers:

.. list-table::
   :header-rows: 1

   * - Marker
     - DK GPIO
     - Meaning
   * - Event
     - P1.12
     - GRTC compare event routed directly through GPPI to a GPIOTE task.
   * - ISR
     - P1.13
     - Start of the GRTC compare callback, after wake-up and interrupt dispatch.
   * - Thread
     - P1.14
     - The blocked application thread has resumed.

Latency mode also writes ``PwrAboveElv`` (value 8) to the ``PIN0`` field of the
private ``SREGS30.PDSELECT`` register. This routes ``pwrAboveElv_a`` to the
dedicated ``PDSELECT0`` pin. The signal is high when the device is not in ELV
mode and low while it is in ELV mode.

Connect the three GPIOs, ``PDSELECT0``, and ground to a logic analyzer. The time
from the Event edge to the ISR edge includes hardware wake-up, exception entry,
and interrupt dispatch. The time from the ISR edge to the Thread edge includes
the remaining interrupt and scheduler processing. The time from Event to Thread
is the application-visible wake-up latency. A low level on ``PDSELECT0`` during
the idle interval confirms that the device entered ELV mode. A normal wake-up
can remain in ELV, so ``PDSELECT0`` is not expected to produce a wake pulse.

The Event edge includes the GRTC-to-GPPI-to-GPIOTE-to-pad propagation delay.
Treat this as measurement-path uncertainty unless a guaranteed timing value is
available for the selected SoC revision. The latency mode also keeps the marker
GPIO and interconnect resources configured, so use the default mode for
unmodified idle-current measurements.

Build the latency configuration with:

.. code-block:: console

   west build -p always -b nrf7120dk/nrf7120/cpuapp nrf/samples/benchmarks/power_consumption -- -DCONFIG_SAMPLE_POWER_CONSUMPTION_LATENCY_MEASUREMENT=y

Constant latency comparison
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The latency measurement uses the SoC's default low-power mode unless
:kconfig:option:`CONFIG_SAMPLE_POWER_CONSUMPTION_CONSTLAT` is enabled. Enabling
this option forces constant latency mode from boot, minimizing and stabilizing
CPU wake-up and PPI task response latency at the cost of increased System ON
Idle power consumption.

Build each target once with the option disabled and once with it enabled, then
compare the Event-to-ISR and Event-to-Thread intervals:

.. code-block:: console

   # nRF54LM20, default low-power mode
   west build -p always -b nrf54lm20dk/nrf54lm20a/cpuapp nrf/samples/benchmarks/power_consumption -- -DCONFIG_SAMPLE_POWER_CONSUMPTION_LATENCY_MEASUREMENT=y

   # nRF54LM20, constant latency mode
   west build -p always -b nrf54lm20dk/nrf54lm20a/cpuapp nrf/samples/benchmarks/power_consumption -- -DCONFIG_SAMPLE_POWER_CONSUMPTION_LATENCY_MEASUREMENT=y -DCONFIG_SAMPLE_POWER_CONSUMPTION_CONSTLAT=y

   # nRF7120, constant latency mode
   west build -p always -b nrf7120dk/nrf7120/cpuapp nrf/samples/benchmarks/power_consumption -- -DCONFIG_SAMPLE_POWER_CONSUMPTION_LATENCY_MEASUREMENT=y -DCONFIG_SAMPLE_POWER_CONSUMPTION_CONSTLAT=y

Sample output
**************

The sample shows the following output:

.. code-block:: console

   Power consumption demo ready.

   Woken up 1 time(s)
   Woken up 2 time(s)
   Woken up 3 time(s)

Dependencies
************

This sample uses the following |NCS| library:

* :ref:`lib_ram_pwrdn`

It uses the following Zephyr library:

* :ref:`zephyr:kernel_api`

In addition, the non-secure build uses the following secure firmware component:

* :ref:`Trusted Firmware-M <ug_tfm>`
