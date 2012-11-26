#!/usr/bin/env python
#=========================================================================
#
#  Copyright Insight Software Consortium
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#         http://www.apache.org/licenses/LICENSE-2.0.txt
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#
#=========================================================================

import SimpleITK as sitk
import sys
import os

if len ( sys.argv ) != 3:
    print "Usage: %s inputImage outputImage" % ( sys.argv[0] )
    sys.exit ( 1 )

inputImage = sitk.ReadImage( sys.argv[1] )

segmenter = sitk.ConfidenceConnectedImageFilter()

segmenter.AddSeed([60,116,100])
segmenter.SetMultiplier( 2.5 )
segmenter.SetNumberOfIterations( 5 )
segmenter.SetReplaceValue( 255 )
segmenter.SetInitialNeighborhoodRadius( 2 )

outputImage = segmenter.Execute( inputImage )

writer = sitk.ImageFileWriter()
writer.SetFileName( sys.argv[2] )
writer.Execute( outputImage )

