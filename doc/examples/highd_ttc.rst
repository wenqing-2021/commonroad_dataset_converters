Custom scenario sampling
=======================================================

Some datasets provide additional meta data for each vehicle, which can be used to select scenarios. The highD dataset
for example provides the minimum time-to-collision (TTC) for each vehicle, indicating criticality of the driving behavior.
To convert scenarios with low TTC of the original trajectory, we customize the conversion process:


.. literalinclude:: ../../examples/highd_ttc.py
   :language: python
   :linenos: