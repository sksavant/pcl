/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#include <pcl/io/png_io.h>
#include <vtkImageImport.h>
#include <vtkPNGWriter.h>
#include <vtkSmartPointer.h>
#include <vtkImageFlip.h>
#include <vtkDoubleArray.h>
#include <vtkRenderer.h>
#include <vtkXYPlotActor.h>
#include <vtkRenderWindow.h>
#include <vtkTextProperty.h>
#include <vtkFieldData.h>
#include <vtkDataObject.h>
#include <vtkWindowToImageFilter.h>
#include <vtkDoubleArray.h>
#include <vtkProperty2D.h>
#include <vtkRendererSource.h>
#include <vtkGraphicsFactory.h>
#include <vtkImagingFactory.h>

namespace 
{
  void flipAndWritePng(const std::string &file_name, vtkSmartPointer<vtkImageImport>& importer)
  {
    vtkSmartPointer<vtkImageFlip> flipYFilter = vtkSmartPointer<vtkImageFlip>::New();
    flipYFilter->SetFilteredAxis(1); // flip y axis
    flipYFilter->SetInputConnection(importer->GetOutputPort());
    flipYFilter->Update();

    vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
    writer->SetFileName(file_name.c_str());
    writer->SetInputConnection(flipYFilter->GetOutputPort());
    writer->Write();
  }

  void renderAndWritePng(const std::string& file_name, vtkSmartPointer<vtkXYPlotActor>& xy_plot, int width, int height)
  {
    // Setup offscreen rendering
    vtkSmartPointer<vtkGraphicsFactory> graphics_factory = vtkSmartPointer<vtkGraphicsFactory>::New();
    graphics_factory->SetOffScreenOnlyMode(1);
    graphics_factory->SetUseMesaClasses(1);

    vtkSmartPointer<vtkImagingFactory> imaging_factory = vtkSmartPointer<vtkImagingFactory>::New();
    imaging_factory->SetUseMesaClasses(1); 

    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->AddActor2D(xy_plot);
    renderer->SetBackground(1,1,1); // Background color white

    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    renderWindow->SetSize(width, height);
    renderWindow->SetBorders(1);
    renderWindow->SetOffScreenRendering(1);
    renderWindow->Render();

    vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
    windowToImageFilter->SetInput(renderWindow);
    windowToImageFilter->SetInputBufferTypeToRGBA(); //also record the alpha (transparency) channel
    windowToImageFilter->Update();

    vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
    writer->SetFileName(file_name.c_str());
    writer->SetInputConnection(windowToImageFilter->GetOutputPort());
    writer->Write();
  }

  void saveHistPNGFile (const std::string& file_name, vtkSmartPointer<vtkDoubleArray>& xy_array, int width, int height)
  {
    vtkSmartPointer<vtkXYPlotActor> xy_plot = vtkSmartPointer<vtkXYPlotActor>::New();
    xy_plot->SetDataObjectPlotModeToColumns();
    xy_plot->SetXValuesToValue();

    vtkSmartPointer<vtkFieldData> field_values = vtkSmartPointer<vtkFieldData>::New();
    field_values->AddArray(xy_array);

    vtkSmartPointer<vtkDataObject> field_data = vtkSmartPointer<vtkDataObject>::New();
    field_data->SetFieldData(field_values);

    xy_plot->AddDataObjectInput(field_data);
    xy_plot->SetPlotColor(0, 1.0, 0.0, 0.0);

    xy_plot->SetDataObjectXComponent(0,0);
    xy_plot->SetDataObjectYComponent(0,1);
    xy_plot->PlotPointsOn();
    xy_plot->PlotCurveLinesOn();

    double min_max[2];
    xy_array->GetRange(min_max, 1);

    xy_plot->SetYTitle (""); xy_plot->SetXTitle ("");
    xy_plot->SetYRange (min_max[0], min_max[1]); 
    xy_plot->SetXRange (0, static_cast<double> (xy_array->GetNumberOfTuples () - 1));

    //renwinint.xy_plot_->SetTitle (id.c_str ());
    xy_plot->GetProperty ()->SetColor (0, 0, 0);

    // Adjust text properties
    vtkSmartPointer<vtkTextProperty> tprop = xy_plot->GetTitleTextProperty ();
    xy_plot->AdjustTitlePositionOn ();
    tprop->SetFontSize (8);
    tprop->ShadowOff (); tprop->ItalicOff ();
    tprop->SetColor (xy_plot->GetProperty ()->GetColor ());

    xy_plot->SetAxisLabelTextProperty (tprop);
    xy_plot->SetAxisTitleTextProperty (tprop);
    xy_plot->SetNumberOfXLabels (8);
    xy_plot->GetProperty ()->SetPointSize (3);
    xy_plot->GetProperty ()->SetLineWidth (2);

    xy_plot->SetPosition (0, 0);
    xy_plot->SetWidth (1); xy_plot->SetHeight (1);

    renderAndWritePng(file_name, xy_plot, width, height);

  }

};



void 
pcl::io::saveCharPNGFile (const std::string &file_name, const unsigned char *char_data, int width, int height, int channels)
{
  vtkSmartPointer<vtkImageImport> importer = vtkSmartPointer<vtkImageImport>::New ();
  importer->SetNumberOfScalarComponents (channels);
  importer->SetDataScalarTypeToUnsignedChar ();
  importer->SetWholeExtent (0, width - 1, 0, height - 1, 0, 0);
  importer->SetDataExtentToWholeExtent ();

  void* data = const_cast<void*> (reinterpret_cast<const void*> (char_data));
  importer->SetImportVoidPointer (data, 1);
  importer->Update ();

  flipAndWritePng(file_name, importer);
}

void 
pcl::io::saveShortPNGFile (const std::string &file_name, const unsigned short *short_image, int width, int height, int channels)
{
  vtkSmartPointer<vtkImageImport> importer = vtkSmartPointer<vtkImageImport>::New ();
  importer->SetNumberOfScalarComponents (channels);
  importer->SetDataScalarTypeToUnsignedShort ();
  importer->SetWholeExtent (0, width - 1, 0, height - 1, 0, 0);
  importer->SetDataExtentToWholeExtent ();

  void* data = const_cast<void*> (reinterpret_cast<const void*> (short_image));
  importer->SetImportVoidPointer (data, 1);
  importer->Update ();

  flipAndWritePng(file_name, importer);
}

void 
pcl::io::saveRgbPNGFile (const std::string& file_name, const unsigned char *rgb_image, int width, int height)
{
  saveCharPNGFile(file_name, rgb_image, width, height, 3);
}

void
pcl::io::savePNGFile (const std::string& file_name, const pcl::PointCloud<unsigned char>& cloud)
{
  saveCharPNGFile(file_name, &cloud.points[0], cloud.width, cloud.height, 1);
}

void
pcl::io::savePNGFile (const std::string& file_name, const pcl::PointCloud<unsigned short>& cloud)
{
  saveShortPNGFile(file_name, &cloud.points[0], cloud.width, cloud.height, 1);
}

void
pcl::io::savePNGFile (const std::string& file_name, const pcl::PCLImage& image)
{
    if (image.encoding == "rgb8")
    {
        saveRgbPNGFile(file_name, &image.data[0], image.width, image.height);
    }
    else if (image.encoding == "mono8")
    {
        saveCharPNGFile(file_name, &image.data[0], image.width, image.height, 1);
    }
    else if (image.encoding == "mono16")
    {
        saveShortPNGFile(file_name, reinterpret_cast<const unsigned short*>(&image.data[0]), image.width, image.height, 1);
    }
    else
    {
        PCL_ERROR ("[pcl::io::savePNGFile] Unsupported image encoding \"%s\".\n", image.encoding.c_str ());
    }
}

void
pcl::io::savePNGFile (const std::string& file_name, const pcl::PointCloud<pcl::PointXYZL>& cloud)
{
	std::vector<unsigned short> data(cloud.width * cloud.height);
	for (size_t i = 0; i < cloud.points.size (); ++i)
	{
		data[i] = static_cast<unsigned short> (cloud.points[i].label);      
	}
	saveShortPNGFile(file_name, &data[0], cloud.width, cloud.height,1);
}

namespace pcl{
    namespace io{
        //template <typename HistT> void
        void
        savePNGFile (const std::string& file_name, const pcl::PointCloud<HistT>& cloud, const std::string& field_name, const int index, int width, int height)
{
  if (index < 0 || index >= cloud.points.size ())
  {
    PCL_ERROR ("[pcl::io::savePNGFile] Invalid point index (%d) given!\n", index);
    return;
  }

  std::vector<pcl::PCLPointField> fields;
  // Check if our field exists
  int field_idx = pcl::getFieldIndex<HistT> (cloud, field_name, fields);
  if (field_idx == -1)
  {
    PCL_ERROR ("[pcl::io::savePNGFile] The specified field <%s> does not exist!\n", field_name.c_str ());
    return;
  }

  vtkSmartPointer<vtkDoubleArray> xy_array = vtkSmartPointer<vtkDoubleArray>::New ();
  xy_array->SetNumberOfComponents (2);
  xy_array->SetNumberOfTuples (fields[field_idx].count);

  // Parse the cloud data and store it in the array
  double xy[2];
  for (uint32_t d = 0; d < fields[field_idx].count; ++d)
  {
    xy[0] = d;
    //xy[1] = cloud.points[index].histogram[d];
    float data;
    memcpy (&data, reinterpret_cast<const char*> (&cloud.points[index]) + fields[field_idx].offset + d * sizeof (float), sizeof (float));
    xy[1] = data;
    xy_array->SetTuple (d, xy);
  }

  saveHistPNGFile(file_name, xy_array, width, height);

}

        //template <typename HistT> void
        void
        savePNGFile (const std::string& file_name, const pcl::PointCloud<HistT>& cloud, const int hsize, int width, int height )
{
  vtkSmartPointer<vtkDoubleArray> xy_array = vtkSmartPointer<vtkDoubleArray>::New ();
  xy_array->SetNumberOfComponents (2);
  xy_array->SetNumberOfTuples (hsize);

  // Parse the cloud data and store it in the array
  double xy[2];
  for (int d = 0; d < hsize; ++d)
  {
    xy[0] = d;
    xy[1] = cloud.points[0].histogram[d];
    xy_array->SetTuple (d, xy);
  }

  saveHistPNGFile(file_name, xy_array, width, height);

}
};
};
