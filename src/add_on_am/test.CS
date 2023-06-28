using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

using System.Linq;
using System.Threading.Tasks;

/// <summary>
/// This class will be instantiated on demand by the Script component.
/// </summary>
public class Script_Instance : GH_ScriptInstance
{
#region Utility functions
  /// <summary>Print a String to the [Out] Parameter of the Script component.</summary>
  /// <param name="text">String to print.</param>
  private void Print(string text) { /* Implementation hidden. */ }
  /// <summary>Print a formatted String to the [Out] Parameter of the Script component.</summary>
  /// <param name="format">String format.</param>
  /// <param name="args">Formatting parameters.</param>
  private void Print(string format, params object[] args) { /* Implementation hidden. */ }
  /// <summary>Print useful information about an object instance to the [Out] Parameter of the Script component. </summary>
  /// <param name="obj">Object instance to parse.</param>
  private void Reflect(object obj) { /* Implementation hidden. */ }
  /// <summary>Print the signatures of all the overloads of a specific method to the [Out] Parameter of the Script component. </summary>
  /// <param name="obj">Object instance to parse.</param>
  private void Reflect(object obj, string method_name) { /* Implementation hidden. */ }
#endregion

#region Members
  /// <summary>Gets the current Rhino document.</summary>
  private readonly RhinoDoc RhinoDocument;
  /// <summary>Gets the Grasshopper document that owns this script.</summary>
  private readonly GH_Document GrasshopperDocument;
  /// <summary>Gets the Grasshopper script component that owns this script.</summary>
  private readonly IGH_Component Component;
  /// <summary>
  /// Gets the current iteration count. The first call to RunScript() is associated with Iteration==0.
  /// Any subsequent call within the same solution will increment the Iteration count.
  /// </summary>
  private readonly int Iteration;
#endregion

  /// <summary>
  /// This procedure contains the user code. Input parameters are provided as regular arguments,
  /// Output parameters as ref arguments. You don't have to assign output parameters,
  /// they will have a default value.
  /// </summary>
  private void RunScript(Mesh M, int contourCount, int iterations, double damping, int smoothing_interval, ref object Contours, ref object Ratio)
  {

    // Initialize variables ##########################################################################
    List<Line>[] contours = new List<Line>[contourCount];
    List<Curve>[] curves = new List<Curve>[contourCount];
    List<Point3d>[] points = new List<Point3d>[contourCount];
    List<Vector3d>[] vectors = new List<Vector3d>[contourCount];
    double[] values = new double[M.Vertices.Count];
    for(int i = 0;i < M.Vertices.Count;i++) values[i] = M.TextureCoordinates[i].Y;
    double lower = values.Min();
    double upper = values.Max();
    double unit_stepsize = (upper - lower) / (contourCount + 1.0);
    double[] iso = new double[contourCount + 1];
    double[] iso_old = new double[contourCount + 1];
    for(int i = 0; i < contourCount; i++) iso[i] = lower + ((i + 1) * unit_stepsize); // Compute iso values for first iteration
    iso_old = iso;
    double[] stepsize_correction = new double[contourCount + 1];
    for(int i = 0; i <= contourCount; i++) stepsize_correction[i] = 1;
    double[] stepsize = new double[contourCount + 1];
    for(int i = 0; i < contourCount; i++) {stepsize[i] = unit_stepsize;} // Compute stepsize for first iteration

    //  Start iterative isoliones generation ###################################################
    for(int iter = 0; iter < iterations; iter++)
    {
      if(iter > 0)
      {
        for(int i = 0; i < contourCount; i++) // Clear all lists
        {
          vectors[i].Clear();
          contours[i].Clear();
          points[i].Clear();
        }
      }

      Parallel.For(0, contourCount, // Compute intersection point for all contour iso values
        j => {
        List<Line> lines = new List<Line>();
        List<Point3d> points_contour = new List<Point3d>();
        List<Vector3d> vecs = new List<Vector3d>();
        for(int i = 0;i < M.Faces.Count;i++) // Check all faces of the mesh if there is a intersection
        {
          int va = M.Faces[i].A;
          int vb = M.Faces[i].B;
          int vc = M.Faces[i].C;
          double a = M.TextureCoordinates[va].Y - iso[j];
          double b = M.TextureCoordinates[vb].Y - iso[j];
          double c = M.TextureCoordinates[vc].Y - iso[j];
          Point3d pa = M.Vertices.Point3dAt(va);
          Point3d pb = M.Vertices.Point3dAt(vb);
          Point3d pc = M.Vertices.Point3dAt(vc);

          List<Point3d> ends = new List<Point3d>();
          if(a < 0 != b < 0)
          {
            double wa = b / (b - a);
            double wb = 1 - wa;
            ends.Add(wa * pa + wb * pb);
          }
          if(a < 0 != c < 0)
          {
            double wa = c / (c - a);
            double wc = 1 - wa;
            ends.Add(wa * pa + wc * pc);
          }
          if(c < 0 != b < 0)
          {
            double wb = c / (c - b);
            double wc = 1 - wb;
            ends.Add(wb * pb + wc * pc);
          }
          if(ends.Count == 2) // if there is a intersection, the line is added to the contour
          {
            lines.Add(new Line(ends[0], ends[1]));
            points_contour.Add(ends[0]);
            points_contour.Add(ends[1]);
            Vector3d vec = new Vector3d((ends[0].X - ends[1].X), (ends[0].Y - ends[1].Y), (ends[0].Z - ends[1].Z));
            vecs.Add(vec);
          }
        }
        contours[j] = lines;
        points[j] = points_contour;
        vectors[j] = vecs;
        });

      // Compute changes in the iso values ###############################################################
      double normalize_factor = 0;
      Rhino.Geometry.Polyline poly;
      double[] distance = new double[contourCount + 1];
      // First the distance to the isoline above and below is computed to know how to change the iso values
      for(int i = 1; i < contourCount - 2; i++){

        poly = new Rhino.Geometry.Polyline(points[i]);
        distance[i] = 0;
        double dist2 = 0;
        for(int j = 0; j < points[i + 1].Count; j++) //distance to isoline above
        {
          distance[i] += Math.Sqrt(points[i + 1][j].DistanceToSquared(poly.ClosestPoint(points[i + 1][j])));
        }
        if(points[i + 1].Count > 0) // Check if isoline (i+1) exists
        {
          stepsize_correction[i] = distance[i] / (points[i + 1].Count);
        }
        poly = new Rhino.Geometry.Polyline(points[i + 1]);
        for(int j = 0; j < points[i].Count; j++) // distance from line below
        {
          dist2 += Math.Sqrt(points[i][j].DistanceToSquared(poly.ClosestPoint(points[i][j])));
        }
        if(points[i].Count > 0 && stepsize_correction[i] > dist2 / (points[i].Count)) // Always choose the smaller one
        {
          stepsize_correction[i] = dist2 / (points[i].Count);
          distance[i] = dist2;
        }
        stepsize_correction[i] = 1 / stepsize_correction[i];
        normalize_factor += stepsize_correction[i]; // Sum up normalize corrections factor
      }

      //For last row - spzial treetment
      distance[contourCount - 1] = 0;
      poly = new Rhino.Geometry.Polyline(points[contourCount - 1]);
      for(int j = 0; j < points[contourCount - 2].Count; j++)
      {
        distance[contourCount - 1] += Math.Sqrt(points[contourCount - 2][j].DistanceToSquared(poly.ClosestPoint(points[contourCount - 2][j]))); //Math.Sqrt
      }
      if(points[contourCount - 1].Count > 0)
      {
        stepsize_correction[contourCount - 1] = distance[contourCount - 1] / points[contourCount - 1].Count;
        stepsize_correction[contourCount - 1] = 1 / stepsize_correction[contourCount - 1];
        normalize_factor += stepsize_correction[contourCount - 1];
      }

      //For first row - spezial treetment
      distance[0] = 0;
      poly = new Rhino.Geometry.Polyline(points[1]);
      for(int j = 0; j < points[0].Count; j++)
      {
        distance[0] += Math.Sqrt(points[0][j].DistanceToSquared(poly.ClosestPoint(points[0][j]))); //Math.Sqrt
      }
      if(points[0].Count > 0)
      {
        stepsize_correction[0] = distance[0] / points[0].Count;
        stepsize_correction[0] = 1 / stepsize_correction[0];
        normalize_factor += stepsize_correction[0];
      }

      normalize_factor = normalize_factor / contourCount;


      for(int i = 0; i < contourCount; i++) // Keep correction value in boundaries
      {
        stepsize_correction[i] = stepsize_correction[i] / normalize_factor;
        if(stepsize_correction[i] > 1.5){stepsize_correction[i] = 1.5;}
        if(stepsize_correction[i] < 0.5){stepsize_correction[i] = 0.5;}
      }
      normalize_factor = normalize_factor / contourCount;

      double sum_steps = 0;

      // Compute correction factors ###########################################################
      for(int i = 0; i < contourCount; i++)
      {
        if(i == 0) // Compute first correction factor
        {
          stepsize[0] = stepsize[0] * stepsize_correction[0];
          iso[0] = lower + stepsize[0];
        }
        else // Compute the other correction factors
          // This process is highly unstable therefore it´s damped
        {
          stepsize_correction[i] = 0.5 * stepsize_correction[i] + 0.5 * stepsize_correction[i - 1];
          stepsize[i] = (1 - damping) * (stepsize[i] * stepsize_correction[i]) + damping * stepsize[i];
        }

        if(iso[i] > upper || iso[i] < lower) // If iso value goes out of range it´s being pushed back to the start value
        {
          iso[i] = lower + ((i + 1) * unit_stepsize);
        }
        sum_steps += stepsize[i];
      }

      // Compute new adapted iso values ######################################################
      double step_correction = unit_stepsize / (sum_steps / (contourCount - 1));

      for(int i = 1; i < contourCount; i++)
      {
        iso[i] = iso[i - 1] + stepsize[i] * step_correction;
      }

      if(iso[0] < lower || iso[0] > lower + unit_stepsize)
      {
        iso[0] = lower + 0.5 * unit_stepsize;
      }
      if(iso[contourCount - 1] > upper || iso[contourCount - 1] < upper - 0.5 * unit_stepsize)
      {
        iso[contourCount - 1] = upper - 0.5 * unit_stepsize;
      }
      iso_old = iso;

      // Depending on the smooting_interval a post smooting is done with a 2 point stencil
      if(iter % smoothing_interval == 0)
      {
        for(int i = 1; i < contourCount - 1 ; i++)
        {
          iso[i] = (iso_old[i + 1] + iso_old[i - 1]) / 2;
        }
      }
    }
    // End iteration ###########################################################################################
    // Analyse output ##########################################################################################
    for(int i = 0; i < contourCount; i++) // Difference of original iso values and adapted isovalues is computed for analysis
    {
      iso[i] = iso[i] - (lower + ((i + 1) * unit_stepsize));
    }
    // analyse = iso;

    double global_max = 0;
    double global_min = 10000;
    List<Point3d> gmax = new List<Point3d>();
    Parallel.For(1, contourCount,
      i => {
      /*
      - Compute minimal distance between all previous points (List) to the lines (List)
      - Local Extremas are the minimum and maximum of the minimal distance values
      - Global Extremas are the Extremas of the local extremas
      */
      double contour_max = 0;
      double contour_min = 10000;
      List<Point3d> max_points = new List<Point3d>();
      for (int j = 0; j < points[i].Count;j++) // Lopp over all points on a contour
      {
        double dist = 10000;
        List<Point3d> lmax_points = new List<Point3d>();
        for (int k = 0; k < points[i - 1].Count; k++) // Compute distance to all points below
        {
          if (dist > Math.Sqrt(points[i][j].DistanceToSquared(points[i - 1][k]))) // choose min as dist
          {
            dist = points[i][j].DistanceToSquared(points[i - 1][k]);
            dist = Math.Sqrt(dist);
            lmax_points.Clear();
            lmax_points.Add(points[i][j]);
            lmax_points.Add(points[i - 1][k]);
          }
        }

        if(contour_max < dist) // Check for Contour min/max
        {
          contour_max = dist;
          max_points.Clear();
          max_points = lmax_points;
        }
        if(contour_min > dist)
        {
          contour_min = dist;
        }
      }

      if(global_max < contour_max) // Check for Global min/max
      {
        global_max = contour_max;
        gmax.Clear();
        gmax = max_points;
      }
      if(global_min > contour_min)
      {
        global_min = contour_min;
      }
      });


    for(int i = 0; i < contourCount; i++)
    {
      List<Line > contour_lines = new List<Line>();
      contour_lines = contours[i];
    }

    // Output ####################################################################

    Ratio = global_max / global_min;
    DataTree<Line> contoursTree = new DataTree<Line>();
    for(int i = 0; i < contourCount;i++)
    {
      GH_Path path = new GH_Path(i);
      for (int j = 0; j < contours[i].Count;j++)
      {
        contoursTree.Add(contours[i][j], path);
      }
    }
    Contours = contoursTree;

  }

  // <Custom additional code> 

  // </Custom additional code> 
}