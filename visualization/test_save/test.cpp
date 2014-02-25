#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>

#include <vtkRenderer.h>
#include <vtkXYPlotActor.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkPNGWriter.h>
#include <vtkTextProperty.h>
#include <vtkFieldData.h>
#include <vtkDataObject.h>
#include <vtkWindowToImageFilter.h>
#include <vtkDoubleArray.h>
#include <vtkProperty2D.h>


int main(){
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_gt(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::io::loadPCDFile<pcl::FPFHSignature33>("fpfhs_cube1.pcd", *fpfhs_gt);

    vtkSmartPointer<vtkDoubleArray> xy_array = vtkSmartPointer<vtkDoubleArray>::New();
    xy_array->SetNumberOfComponents(2);
    xy_array->SetNumberOfTuples(33); // (cloud.width? no!)hsize; cloud.fields[field_idx].count
    double xy[2];
    for (unsigned int d=0; d<33; ++d){
        xy[0] = d;
        xy[1] = fpfhs_gt->points[0].histogram[d];
        xy_array->SetTuple(d,xy);
        /*
        float data;
        memcpy(&data, &cloud. PC2
        */
    }

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


    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->AddActor2D(xy_plot);
    renderer->SetBackground(1,1,1); // Background color white

    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    renderWindow->Render();

    vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
    windowToImageFilter->SetInput(renderWindow);
    //windowToImageFilter->SetMagnification(3); //set the resolution of the output image (3 times the current resolution of vtk render window)
    windowToImageFilter->SetInputBufferTypeToRGBA(); //also record the alpha (transparency) channel
    windowToImageFilter->Update();

    vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
    writer->SetFileName("fpfhs_1.png");
    writer->SetInputConnection(windowToImageFilter->GetOutputPort());
    writer->Write();
    return 0;

}
