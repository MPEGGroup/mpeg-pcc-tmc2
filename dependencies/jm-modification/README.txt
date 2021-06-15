######PCC_JM.patch######
"PCC_JM.patch" is provided by ITRI, which makes some changes of the AVC codec (JM) to integrate it as the internal library of TMC2.

######SampleStreamITRI.patch######
"SampleStreamITRI.patch" is provided by ITRI, which enables the AVC codec (JM) to support the functionality of sample stream format.
To active this functionality, applying the "SampleStreamITRI.patch" is needed:
- For the usage of the codec option of external AVC application (JMAPP):
  Unmark the patch-applying command in the cmake file "/dependencies/cmake/jm_app.cmake".
- For the usage of the codec option of internal AVC library (JMLIB):
  Unmark the patch-applying command in the cmake file "/dependencies/cmake/jm_lib.cmake".