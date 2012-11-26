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

if len ( sys.argv ) != 3:
    print "Usage: %s inputImage outputImage" % ( sys.argv[0] )
    sys.exit ( 1 )

inputImage = sitk.ReadImage( sys.argv[1] )

canny = sitk.CannyEdgeDetectionImageFilter()

canny.SetLowerThreshold( 1 )
canny.SetUpperThreshold( 30 )
canny.SetVariance( [ 2.0, 2.0, 2.0] )

caster = sitk.CastImageFilter()
caster.SetOutputPixelType(sitk.sitkFloat32)

floatImage = caster.Execute( inputImage )
outputImage = canny.Execute( floatImage )

writer = sitk.ImageFileWriter()
writer.SetFileName( sys.argv[2] )
writer.Execute( outputImage )

