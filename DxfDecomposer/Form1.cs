using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO;
using static System.Console;
using netDxf;

namespace DxfDecomposer
{
    public partial class Form1 : Form
    {
		private void decompose_DXF(string dxfpath, string txtpath, int prec)
		{
			DxfDocument dxf;
			try
			{
				dxf = DxfDocument.Load(dxfpath);
			}
			catch (Exception ex)
			{
				MessageBox.Show("dxf load fail.\r\n" + ex.Message);
				return;
			}
			if (dxf == null)
			{
				MessageBox.Show("dxf load fail.");
				return;
			}
			//Save the coordinates of each segment into a list
			List<List<List<double>>> doublelist = new List<List<List<double>>>();
			//LwPolylines
			if (dxf.LwPolylines.Count != 0)
			{
				foreach (netDxf.Entities.LwPolyline i in dxf.LwPolylines)
				{
					List<netDxf.Entities.EntityObject> polymember = new List<netDxf.Entities.EntityObject>();
					polymember = i.Explode();
					dxf.AddEntity(polymember);
				}
			}
			//Splines
			if (dxf.Splines.Count != 0)
			{

				foreach (netDxf.Entities.Spline i in dxf.Splines)
				{
					List<List<double>> element = new List<List<double>>();
					netDxf.Entities.Polyline k = i.ToPolyline(20);
					foreach (netDxf.Entities.PolylineVertex j in k.Vertexes)
					{
						List<double> coor = new List<double>();
						coor.Add(j.Location.X);
						coor.Add(j.Location.Y);
						element.Add(coor);
					}
					doublelist.Add(element);
				}
			}
			//Arcs
			if (dxf.Arcs.Count != 0)
			{
				foreach (netDxf.Entities.Arc i in dxf.Arcs)
				{
					List<List<double>> element = new List<List<double>>();
					//int sampling_point=(int)(Math.Round(Math.Abs(i.EndAngle - i.StartAngle) / prec)+0.5);
					//netDxf.Entities.LwPolyline k = i.ToPolyline(sampling_point);
					int number_of_sampling = (int)Math.Round(2 * Math.PI * i.Radius * Math.Min(Math.Abs(i.StartAngle - i.EndAngle), Math.Abs(i.StartAngle - i.EndAngle - 360)) / 360 * (1 / prec));
					if (number_of_sampling < 2) number_of_sampling = 2;
					netDxf.Entities.LwPolyline k = i.ToPolyline(number_of_sampling);
					foreach (netDxf.Entities.LwPolylineVertex j in k.Vertexes)
					{
						List<double> coor = new List<double>();
						coor.Add(j.Location.X);
						coor.Add(j.Location.Y);
						element.Add(coor);
					}
					doublelist.Add(element);
				}
			}
			//Circles
			if (dxf.Circles.Count != 0)
			{
				foreach (netDxf.Entities.Circle i in dxf.Circles)
				{
					List<List<double>> element = new List<List<double>>();
					int number_of_sampling = (int)Math.Round(2 * Math.PI * i.Radius * (1 / prec));
					netDxf.Entities.LwPolyline k = i.ToPolyline(number_of_sampling);
					foreach (netDxf.Entities.LwPolylineVertex j in k.Vertexes)
					{
						List<double> coor = new List<double>();
						coor.Add(j.Location.X);
						coor.Add(j.Location.Y);
						element.Add(coor);
					}
					doublelist.Add(element);
				}
			}
			//Lines
			if (dxf.Lines.Count != 0)
			{
				foreach (netDxf.Entities.Line i in dxf.Lines)
				{
					/*List<List<double>> element = new List<List<double>>();
					List<double> coor1 = new List<double>();
					coor1.Add(i.StartPoint.X);
					coor1.Add(i.StartPoint.Y);
					element.Add(coor1);
					List<double> coor2 = new List<double>();
					coor2.Add(i.EndPoint.X);
					coor2.Add(i.EndPoint.Y);
					element.Add(coor2);
					doublelist.Add(element);*/
					List<List<double>> element = new List<List<double>>();
					double m = i.StartPoint.X;
					double n = i.StartPoint.Y;
					double s = i.EndPoint.X;
					double t = i.EndPoint.Y;
					double x = s - m;
					double y = t - n;
					//Normalize
					double n_x = x / Math.Sqrt(x * x + y * y) * prec;
					double n_y = y / Math.Sqrt(x * x + y * y) * prec;
					int number_of_sampling = (int)(Math.Floor(Math.Sqrt(x * x + y * y) / prec));
					for (int j = 0; j <= number_of_sampling; j++)
					{
						List<double> coor1 = new List<double>();
						coor1.Add(m + n_x * j);
						coor1.Add(n + n_y * j);
						element.Add(coor1);
					}
					List<double> coor2 = new List<double>();
					coor2.Add(i.EndPoint.X);
					coor2.Add(i.EndPoint.Y);
					element.Add(coor2);
					doublelist.Add(element);
				}
			}
			//Sorting
			List<List<List<double>>> result = new List<List<List<double>>>();
			while (doublelist.Count != 0)
			{
				List<List<double>> currentoutline = new List<List<double>>();
				currentoutline = doublelist[0];
				List<double> currentend = new List<double>();
				currentend = currentoutline[currentoutline.Count - 1];
				doublelist.RemoveAt(0);

				int size = doublelist.Count;
				for (int j = 0; j < size; j++)
				{
					for (int i = 0; i < doublelist.Count; i++)
					{
						List<double> newhead = new List<double>();
						newhead = doublelist[i][0];

						List<double> newend = new List<double>();
						newend = (doublelist[i][doublelist[i].Count - 1]);
						if (Math.Abs(currentend[0] - newhead[0]) < 0.05 && Math.Abs(currentend[1] - newhead[1]) < 0.05)
						{
							int tail = currentoutline.Count - 1;
							currentoutline.AddRange(doublelist[i]);
							currentoutline.RemoveAt(tail);
							currentend.RemoveAt(0);
							currentend = currentoutline[currentoutline.Count - 1];
							doublelist.RemoveAt(i);
							break;
						}
						else if (Math.Abs(currentend[0] - newend[0]) < 0.05 && Math.Abs(currentend[1] - newend[1]) < 0.05)
						{
							int tail = currentoutline.Count - 1;
							doublelist[i].Reverse();
							currentoutline.AddRange(doublelist[i]);
							currentoutline.RemoveAt(tail);
							currentend.RemoveAt(0);
							currentend = currentoutline[currentoutline.Count - 1];
							doublelist.RemoveAt(i);
							break;
						}
					}
				}
				//currentoutline.Add(currentoutline[0]);
				//currentoutline.Add(currentoutline[1]);
				result.Add(currentoutline);
			}
			List<List<double>> outline = new List<List<double>>();
			List<List<List<double>>> inneroutline = new List<List<List<double>>>();
			outline = result[0];
			double maxarea = 0;
			List<double> center_and_boundary = new List<double>();
			for (int i = 0; i < result.Count; i++)
			{
				double right = result[i][0][0];
				double left = result[i][0][0];
				double top = result[i][0][1];
				double bottom = result[i][0][1];
				for (int j = 0; j < result[i].Count; j++)
				{
					if (result[i][j][0] > right) right = result[i][j][0];
					if (result[i][j][0] < left) left = result[i][j][0];
					if (result[i][j][1] > top) top = result[i][j][1];
					if (result[i][j][1] < bottom) bottom = result[i][j][1];

				}
				double currentarea = (top - bottom) * (right - left);
				if (i == 0)
				{
					maxarea = currentarea;
					outline = result[i];
					center_and_boundary.Add((right + left) / 2);
					center_and_boundary.Add((top + bottom) / 2);
					center_and_boundary.Add(left);
					center_and_boundary.Add(top);
					center_and_boundary.Add(right);
					center_and_boundary.Add(bottom);

				}
				else if (currentarea > maxarea && i > 0)
				{
					maxarea = currentarea;
					inneroutline.Add(outline);
					outline = result[i];
					center_and_boundary[0] = ((right + left) / 2);
					center_and_boundary[1] = ((top + bottom) / 2);
					center_and_boundary[2] = left;
					center_and_boundary[3] = top;
					center_and_boundary[4] = right;
					center_and_boundary[5] = bottom;
				}
				else
				{
					inneroutline.Add(result[i]);
				}
			}
			StreamWriter str = new StreamWriter(txtpath);
			foreach (double j in center_and_boundary)
			{

				str.WriteLine(j.ToString());

			}
			str.WriteLine("");
			foreach (List<double> j in outline)
			{
				foreach (double k in j)
				{
					str.WriteLine(k.ToString());
				}
			}
			str.WriteLine("");
			foreach (List<List<double>> i in inneroutline)
			{
				foreach (List<double> j in i)
				{
					foreach (double k in j)
					{
						str.WriteLine(k.ToString());
					}
				}
				str.WriteLine("");
			}
			str.Close();
		}
		public Form1()
        {
            InitializeComponent();
        }
		[STAThread]

		private void button2_Click(object sender, EventArgs e)
        {
			OpenFileDialog openFileDialog1 = new OpenFileDialog();
			openFileDialog1.Title = "選擇要開啟的Dxf檔案";
			if (openFileDialog1.ShowDialog() == System.Windows.Forms.DialogResult.OK)
			{
				string @Filename = openFileDialog1.FileName; //取得檔名
				decompose_DXF(Filename, @"C:\Users\wades\Desktop\OpenSuorce\Contour.txt", 20);
			}

        }
    }
}
