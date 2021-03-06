/*=========================================================================
 *
 *  Copyright Insight Software Consortium
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0.txt
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *=========================================================================*/

#include "itkConfidenceConnectedImageFilter.h"
#include "itkCurvatureFlowImageFilter.h"

#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"


int main( int argc, char *argv[] )
{

  if( argc < 5 )
    {
    std::cerr << "Missing Parameters " << std::endl;
    std::cerr << "Usage: " << argv[0];
    std::cerr << " inputImage  outputImage seedX seedY " << std::endl;
    return 1;
    }


  typedef   float           InternalPixelType;
  const     unsigned int    Dimension = 2;
  typedef itk::Image< InternalPixelType, Dimension >  InputImageType;

  typedef unsigned char                            OutputPixelType;
  typedef itk::Image< OutputPixelType, Dimension > OutputImageType;

  typedef  itk::ImageFileReader< InputImageType  > ReaderType;
  typedef  itk::ImageFileWriter< OutputImageType > WriterType;

  ReaderType::Pointer reader = ReaderType::New();
  WriterType::Pointer writer = WriterType::New();

  reader->SetFileName( argv[1] );
  writer->SetFileName( argv[2] );

  typedef itk::ConfidenceConnectedImageFilter<
    InputImageType, OutputImageType> ConnectedFilterType;

  ConnectedFilterType::Pointer confidenceConnected = ConnectedFilterType::New();

  confidenceConnected->SetInput( reader->GetOutput() );
  writer->SetInput( confidenceConnected->GetOutput() );

  confidenceConnected->SetMultiplier( 2.5 );
  confidenceConnected->SetNumberOfIterations( 5 );
  confidenceConnected->SetReplaceValue( 255 );
  confidenceConnected->SetInitialNeighborhoodRadius( 2 );

  InputImageType::IndexType  index;

  index[0] = atoi( argv[3] );
  index[1] = atoi( argv[4] );

  confidenceConnected->SetSeed( index );

  try
    {
    writer->Update();
    }
  catch( itk::ExceptionObject & excep )
    {
    std::cerr << "Exception caught !" << std::endl;
    std::cerr << excep << std::endl;
    }

  return EXIT_SUCCESS;
}
