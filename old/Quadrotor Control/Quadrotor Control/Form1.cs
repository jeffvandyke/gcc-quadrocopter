﻿using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace Quadrotor_Control
{
   public partial class Form1 : Form
   {
      private SerialPort bluetooth;
      private Pen connectionStatusPen;
      private Rectangle connectionStatusRectangle;

      // need to keep a flag to know when to ignore Left/Right presses because editing text
      private bool editingText;

      // need to know if a path for the log files is selected
      private bool pathSelected;

      // flag for if user wants to write log files
      private bool writeToFile;

      // streamwriter to log data received
      private StreamWriter streamWriter;

      // latest file to have been opened
      private String latestFileName;

      public Form1()
      {
         // initialize it to null
         this.streamWriter = null;
         this.latestFileName = "";

         InitializeComponent();

         // initialize flags
         editingText = false;
         pathSelected = false;
         writeToFile = false;

         // set location of the connectionStatusRectangle
         this.connectionStatusRectangle = new Rectangle(this.connectionStatusLabel.Location, new Size(10, 10));
         connectionStatusRectangle.Offset(45, 0);

         // set color of connectionStatusPen to be Red since by default disconnected
         this.connectionStatusPen = new Pen(Color.Red, 10);
         
         // populate port selection box
         foreach (var port in SerialPort.GetPortNames())
         {
            portComboBox.Items.Add(port);
         }

         // setup bluetooth connection
         bluetooth = new SerialPort("COM", 115200, Parity.None, 8, StopBits.One);
         
         this.DrawConnectionStatus();
      }

      private void DataReceivedHandler(object sender, SerialDataReceivedEventArgs e)
      {
          SerialPort sp = (SerialPort)sender;
          try
          {
              string message = sp.ReadLine();
              DataReceived(message);
          }
          catch (Exception)
          {
 
          }         
      }

      // This delegate enables asynchronous calls for setting 
      // the text property on a TextBox control. 
      delegate void SetTextCallback(string text);

      private void DataReceived(string message)
      {
         // InvokeRequired required compares the thread ID of the 
         // calling thread to the thread ID of the creating thread. 
         // If these threads are different, it returns true. 
         if (this.receivedDataTextBox.InvokeRequired)
         {
            SetTextCallback d = new SetTextCallback(SetReceivedText);
            this.Invoke(d, new object[] { message });
         }
         else
         {
            this.SetReceivedText(message);
         }
      }

      private void SetReceivedText(string text)
      {
         this.receivedDataTextBox.AppendText(text + Environment.NewLine);
         if (writeToFile) streamWriter.Write(text); // add text to log file
      }

      private void connectionButton_Click(object sender, EventArgs e)
      {
         if (portComboBox.SelectedItem == null)
         {
            MessageBox.Show("Port not selected");
            return;
         }

         try
         {
            // connect via port selected
            bluetooth.PortName = portComboBox.SelectedItem.ToString();
            
            // open connection to quadrotor bluetooth
            bluetooth.Open();
            bluetooth.ReadTimeout = 1000;
            bluetooth.WriteTimeout = 1000;
            // connect to device
            bluetooth.WriteLine("C,0006666007BA");

            // change color of connection status
            this.DrawConnectionStatus();

            // start reading
            bluetooth.DataReceived += new SerialDataReceivedEventHandler(DataReceivedHandler);
         }
         catch (Exception ex)
         {
            MessageBox.Show(String.Format("{0}: {1}", ex.GetType().Name, ex.Message));
         }
      }

      private void Disconnect()
      {
         if (writeToFile) streamWriter.Close(); // close the file if it was opened
         try
         {
            // close connection
            bluetooth.Close();

            // change color of connection status
            this.DrawConnectionStatus();
         }
         catch (NullReferenceException ex)
         {
            // ignore this since it probably occured because connect was never pressed
         }
         catch (Exception ex)
         {
            MessageBox.Show(String.Format("{0}: {1}", ex.GetType().Name, ex.Message));
         }
      }

      private void disconnectButton_Click(object sender, EventArgs e)
      {
         this.Disconnect();
      }

      private void DrawConnectionStatus()
      {
         // Red = closed   Green = open
         this.connectionStatusPen.Color = bluetooth.IsOpen ? Color.Green : Color.Red;
         Rectangle redrawArea = new Rectangle(this.connectionStatusRectangle.Location, new Size(30, 30));
         redrawArea.Offset(-10, -10);
         this.Invalidate(redrawArea); // force screen to redraw indicator
      }

      private void Form1_Paint(object sender, PaintEventArgs e)
      {
         // Draw connectionStatus to screen.
         e.Graphics.DrawEllipse(this.connectionStatusPen, this.connectionStatusRectangle);
      }

      private void clearReceivedButton_Click(object sender, EventArgs e)
      {
         this.receivedDataTextBox.Clear();
      }

      private void clearTransmitButton_Click(object sender, EventArgs e)
      {
         this.transmittedDataTextBox.Clear();
      }

      private void sendButton_Click(object sender, EventArgs e)
      {
         try
         {
            this.SendDataAndLog(sendTextBox.Text);
         }
         catch (TimeoutException ex)
         {
            transmittedDataTextBox.AppendText(String.Format("{0}: {1}", ex.GetType().Name, ex.Message) + Environment.NewLine);
         }
         catch (Exception ex)
         {
            MessageBox.Show(String.Format("{0}: {1}", ex.GetType().Name, ex.Message));
         }
      }

      // transmit data and add to textbox
      private void SendDataAndLog(string message)
      {
         bluetooth.WriteLine(message);
         transmittedDataTextBox.AppendText(message + Environment.NewLine);
      }

      // process keystrokes to send messages
      protected override bool ProcessCmdKey(ref Message msg, Keys keyData)
      {
         if (bluetooth.IsOpen)
         {
            switch (keyData)
            {
               case Keys.Escape:
                  this.SendDataAndLog("Escape");
                  return true;    // indicate that you handled this keystroke
               case Keys.Space:
                  this.SendDataAndLog("Start");
                  return true;
               case Keys.Right:
                  if (editingText) return false;
                  this.SendDataAndLog("Right");
                  return true;
               case Keys.Left:
                  if (editingText) return false;
                  this.SendDataAndLog("Left");
                  return true;
               case Keys.Up:
                  this.SendDataAndLog("Up");
                  return true;
               case Keys.Down:
                  this.SendDataAndLog("Down");
                  return true;
               default:
                  break;
            }
         }

         // Call the base class
         return base.ProcessCmdKey(ref msg, keyData);
      }

      private void sendTextBox_Enter(object sender, EventArgs e)
      {
         // when entering text
         editingText = true;
      }

      private void sendTextBox_Leave(object sender, EventArgs e)
      {
         // stopped entering text
         editingText = false;
      }

      private void Form1_FormClosing(object sender, FormClosingEventArgs e)
      {
         this.Disconnect();
      }

      private void startNewFileButton_Click(object sender, EventArgs e)
      {
         // if file already opened - close existing
         if (streamWriter != null) streamWriter.Close();

         if (pathSelected)
         {
            writeToFile = true;
            string filename = String.Format(currentPathLabel.Text + "\\Data-" + "{0:MM-dd-yyyy-hh-mm-tt}" + ".txt", DateTime.Now);
            streamWriter = new StreamWriter(filename);
            filenameLabel.Text = filename;
            latestFileName = filename;
            matlabButton.Enabled = true;
         }
         else
         {
            MessageBox.Show("No path selected");
         }
         
      }

      private void selectLogFilePathButton_Click(object sender, EventArgs e)
      {
         FolderBrowserDialog dialog = new FolderBrowserDialog();

         if (dialog.ShowDialog() == DialogResult.OK)
         {
            currentPathLabel.Text = dialog.SelectedPath;
            pathSelected = true;
         }
      }

      private void closeFileButton_Click(object sender, EventArgs e)
      {
         if (streamWriter != null) streamWriter.Close();
         filenameLabel.Text = "";
      }

      //private void matlabButton_Click(object sender, EventArgs e)
      //{
      //   MLApp.MLApp matlab = new MLApp.MLApp();
      //   System.Threading.Thread.Sleep(3000); // wait for matlab to open
      //   // find the directory of the matlab test script
      //   String dir = Directory.GetCurrentDirectory().ToString();
      //   dir = dir.Substring(0, dir.IndexOf("\\trunk") + 6) + "\\Quadrotor";
      //   String cmd = "cd " + dir.Replace("\\", "\\\\");
      //   matlab.Execute(cmd);
      //   String name = latestFileName.Replace("\\", "\\\\");
      //   String script = "PlotMultipleTests('" + name + "')";
      //   // close the file so that matlab can use it
      //   if (streamWriter != null) streamWriter.Close();
      //   filenameLabel.Text = "";
     
      //   String result = matlab.Execute(script);
      //}
   }
}
