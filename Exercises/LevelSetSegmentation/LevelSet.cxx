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

#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkLevelSetDomainMapImageFilter.h"
#include "itkLevelSetContainer.h"
#include "itkLevelSetEquationChanAndVeseInternalTerm.h"
#include "itkLevelSetEquationChanAndVeseExternalTerm.h"
#include "itkLevelSetEquationTermContainer.h"
#include "itkLevelSetEquationContainer.h"
#include "itkSinRegularizedHeavisideStepFunction.h"
#include "itkLevelSetEvolution.h"
#include "itkBinaryImageToLevelSetImageAdaptor.h"
#include "itkLevelSetEvolutionNumberOfIterationsStoppingCriterion.h"
#include "itkNumericTraits.h"
#include "itkWhitakerSparseLevelSetImage.h"
#include "itkLevelSetEquationCurvatureTerm.h"


int main( int argc, char* argv[] )
{
  if( argc < 5 )
    {
    std::cerr << "Missing Arguments" << std::endl;
    std::cerr << "./LevelSetExercise1 " <<std::endl;
    std::cerr << "1- Input Image" <<std::endl;
    std::cerr << "2- Number of Iterations" <<std::endl;
    std::cerr << "3- Curvature Term coefficient" <<std::endl;
    std::cerr << "4- Output Image" <<std::endl;

    return EXIT_FAILURE;
    }

  // Image Dimension
  const unsigned int Dimension = 2;

  typedef unsigned char                                     InputPixelType;
  typedef itk::Image< InputPixelType, Dimension >           InputImageType;

  // Read input image (to be processed).
  typedef itk::ImageFileReader< InputImageType >            ReaderType;
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName( argv[1] );
  reader->Update();
  InputImageType::Pointer inputImage = reader->GetOutput();

  // Generate a binary mask that will be used as initialization
  // of the level set evolution.
  InputImageType::Pointer binary = InputImageType::New();
  binary->SetRegions( inputImage->GetLargestPossibleRegion() );
  binary->CopyInformation( inputImage );
  binary->Allocate();
  binary->FillBuffer( itk::NumericTraits<InputPixelType>::Zero );

  InputImageType::RegionType region = inputImage->GetBufferedRegion();
  InputImageType::IndexType index;
  InputImageType::SizeType size;

  const unsigned int borderSize = 5;

  index.Fill( borderSize );
  size = region.GetSize();

  size[0] -= 2 * borderSize;
  size[1] -= 2 * borderSize;

  region.SetIndex( index );
  region.SetSize( size );

  typedef itk::ImageRegionIteratorWithIndex< InputImageType > InputIteratorType;
  InputIteratorType iIt( binary, region );
  iIt.GoToBegin();
  while( !iIt.IsAtEnd() )
    {
    iIt.Set( itk::NumericTraits<InputPixelType>::One );
    ++iIt;
    }

  // Convert the binary mask into a level set function.
  // Here the output level-set will be a "Whitaker" sparse level-set;
  // i.e. only few layers {-2, -1, 0, +1, +2 } around the zero-set are
  // maintained, the rest of the domain is either -3 or +3.
  typedef float PixelType;

  typedef itk::WhitakerSparseLevelSetImage< PixelType, Dimension > WhitakerSparseLevelSetImageType;

  typedef itk::BinaryImageToLevelSetImageAdaptor< InputImageType,
    WhitakerSparseLevelSetImageType > BinaryToSparseAdaptorType;

  BinaryToSparseAdaptorType::Pointer adaptor = BinaryToSparseAdaptorType::New();
  adaptor->SetInputImage( binary );
  adaptor->Initialize();
  std::cout << "Finished converting to sparse format" << std::endl;

  // Here get the resulting level-set function
  typedef BinaryToSparseAdaptorType::LevelSetType SparseLevelSetType;

  SparseLevelSetType::Pointer levelSet = adaptor->GetLevelSet();

  // Create here the bounds in which this level-set can evolved.

  // There is only one level-set, so we fill 1 list with only one element which
  // correspondongs to the level-set identifier.
  typedef itk::IdentifierType         IdentifierType;
  typedef std::list< IdentifierType > IdListType;

  IdListType list_ids;
  list_ids.push_back( 1 );

  // We create one image where for each pixel we provide which level-set exists.
  // In this example the first level-set is defined on the whole image.
  typedef itk::Image< IdListType, Dimension >               IdListImageType;
  IdListImageType::Pointer id_image = IdListImageType::New();
  id_image->SetRegions( inputImage->GetLargestPossibleRegion() );
  id_image->Allocate();
  id_image->FillBuffer( list_ids );

  typedef itk::Image< short, Dimension >                     CacheImageType;
  typedef itk::LevelSetDomainMapImageFilter< IdListImageType, CacheImageType >
                                                            DomainMapImageFilterType;
  DomainMapImageFilterType::Pointer domainMapFilter = DomainMapImageFilterType::New();
  domainMapFilter->SetInput( id_image );
  domainMapFilter->Update();
  std::cout << "Domain map computed" << std::endl;

  // Define the Heaviside function
  typedef SparseLevelSetType::OutputRealType LevelSetOutputRealType;

  typedef itk::SinRegularizedHeavisideStepFunction< LevelSetOutputRealType,
      LevelSetOutputRealType > HeavisideFunctionBaseType;
  HeavisideFunctionBaseType::Pointer heaviside = HeavisideFunctionBaseType::New();
  heaviside->SetEpsilon( 1.0 );

  // Insert the levelsets in a levelset container
  typedef itk::LevelSetContainer< IdentifierType, SparseLevelSetType >
      LevelSetContainerType;

  LevelSetContainerType::Pointer lscontainer = LevelSetContainerType::New();
  lscontainer->SetHeaviside( heaviside );
  lscontainer->SetDomainMapFilter( domainMapFilter );

  lscontainer->AddLevelSet( 0, levelSet );

  std::cout << "Level set container created" << std::endl;

  // **************** CREATE ALL TERMS ****************

  // Create ChanAndVese internal term for phi
  typedef itk::LevelSetEquationChanAndVeseInternalTerm< InputImageType,
      LevelSetContainerType > ChanAndVeseInternalTermType;

  ChanAndVeseInternalTermType::Pointer cvInternalTerm0 = ChanAndVeseInternalTermType::New();
  cvInternalTerm0->SetInput( inputImage );
  cvInternalTerm0->SetCoefficient( 1.0 );
  cvInternalTerm0->SetCurrentLevelSetId( 0 );
  cvInternalTerm0->SetLevelSetContainer( lscontainer );
  std::cout << "Chan and Vese internal term created" << std::endl;

  // Create ChanAndVese external term for phi
  typedef itk::LevelSetEquationChanAndVeseExternalTerm< InputImageType,
      LevelSetContainerType > ChanAndVeseExternalTermType;

  ChanAndVeseExternalTermType::Pointer cvExternalTerm0 = ChanAndVeseExternalTermType::New();
  cvExternalTerm0->SetInput( inputImage );
  cvExternalTerm0->SetCoefficient( 1.0 );
  cvExternalTerm0->SetCurrentLevelSetId( 0 );
  cvExternalTerm0->SetLevelSetContainer( lscontainer );
  std::cout << "Chan and Vese external term created" << std::endl;

  // A good value to try; 4000.0
  double CurvatureTermCoefficient = atof( argv[3] );
  std::cout <<"CurvatureTermCoefficient : "
            << CurvatureTermCoefficient <<std::endl;

  // put the curvatre term here!
  typedef itk::LevelSetEquationCurvatureTerm<
    InputImageType, LevelSetContainerType > CurvatureTermType;

  CurvatureTermType::Pointer curvatureTerm = CurvatureTermType::New();
  curvatureTerm->SetInput( binary );
  curvatureTerm->SetCoefficient( CurvatureTermCoefficient );
  curvatureTerm->SetCurrentLevelSetId( 0 );
  curvatureTerm->SetLevelSetContainer( lscontainer );


  // **************** CREATE ALL EQUATIONS ****************

  // Create Term Container which corresponds to the combination of terms in the PDE.
  typedef itk::LevelSetEquationTermContainer< InputImageType, LevelSetContainerType >
                                                            TermContainerType;
  TermContainerType::Pointer termContainer0 = TermContainerType::New();
  termContainer0->SetInput( inputImage );
  termContainer0->SetLevelSetContainer( lscontainer );

  termContainer0->AddTerm( 0, cvInternalTerm0 );
  termContainer0->AddTerm( 1, cvExternalTerm0 );
  termContainer0->AddTerm( 2, curvatureTerm );

  std::cout << "Term container 0 created" << std::endl;

  typedef itk::LevelSetEquationContainer< TermContainerType >
                                                            EquationContainerType;
  EquationContainerType::Pointer equationContainer = EquationContainerType::New();
  equationContainer->AddEquation( 0, termContainer0 );
  equationContainer->SetLevelSetContainer( lscontainer );

  typedef itk::LevelSetEvolutionNumberOfIterationsStoppingCriterion< LevelSetContainerType >
      StoppingCriterionType;
  StoppingCriterionType::Pointer criterion = StoppingCriterionType::New();
  criterion->SetNumberOfIterations( atoi( argv[2]) );

  typedef itk::LevelSetEvolution< EquationContainerType, SparseLevelSetType > LevelSetEvolutionType;

  LevelSetEvolutionType::Pointer evolution = LevelSetEvolutionType::New();

  evolution->SetEquationContainer( equationContainer );
  evolution->SetStoppingCriterion( criterion );
  evolution->SetLevelSetContainer( lscontainer );

  try
    {
    evolution->Update();
    }
  catch ( itk::ExceptionObject& err )
    {
    std::cerr << err << std::endl;
    return EXIT_FAILURE;
    }

  typedef signed char  OutputPixelType;
  typedef itk::Image< OutputPixelType, Dimension > OutputImageType;
  OutputImageType::Pointer outputImage = OutputImageType::New();
  outputImage->SetRegions( inputImage->GetLargestPossibleRegion() );
  outputImage->CopyInformation( inputImage );
  outputImage->Allocate();
  outputImage->FillBuffer( 0 );

  typedef itk::ImageRegionIteratorWithIndex< OutputImageType > OutputIteratorType;
  OutputIteratorType oIt( outputImage, outputImage->GetLargestPossibleRegion() );
  oIt.GoToBegin();

  OutputImageType::IndexType idx;

  while( !oIt.IsAtEnd() )
    {
    idx = oIt.GetIndex();
    oIt.Set( levelSet->GetLabelMap()->GetPixel(idx) );
    ++oIt;
    }

  typedef unsigned char WritePixelType;
  typedef itk::Image< WritePixelType, Dimension > WriteImageType;

  typedef itk::BinaryThresholdImageFilter<
    OutputImageType, WriteImageType > ThresholdFilterType;

  ThresholdFilterType::Pointer thresholder = ThresholdFilterType::New();

  thresholder->SetInput( outputImage );
  thresholder->SetOutsideValue( itk::NumericTraits< WritePixelType >::max() );
  thresholder->SetInsideValue(  itk::NumericTraits< WritePixelType >::Zero );
  thresholder->SetLowerThreshold( itk::NumericTraits< OutputPixelType >::Zero );
  thresholder->SetUpperThreshold( itk::NumericTraits< OutputPixelType >::max() );

  typedef itk::ImageFileWriter< WriteImageType >   OutputWriterType;
  OutputWriterType::Pointer writer = OutputWriterType::New();
  writer->SetFileName( argv[4] );
  writer->SetInput( thresholder->GetOutput() );

  try
    {
    writer->Update();
    }
  catch ( itk::ExceptionObject& err )
    {
    std::cout << err << std::endl;
    }

  return EXIT_SUCCESS;
}
