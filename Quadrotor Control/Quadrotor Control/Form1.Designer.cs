namespace Quadrotor_Control
{
   partial class Form1
   {
      /// <summary>
      /// Required designer variable.
      /// </summary>
      private System.ComponentModel.IContainer components = null;

      /// <summary>
      /// Clean up any resources being used.
      /// </summary>
      /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
      protected override void Dispose(bool disposing)
      {
         if (disposing && (components != null))
         {
            this.bluetooth.Close();
            components.Dispose();
         }
         base.Dispose(disposing);
      }

      #region Windows Form Designer generated code

      /// <summary>
      /// Required method for Designer support - do not modify
      /// the contents of this method with the code editor.
      /// </summary>
      private void InitializeComponent()
      {
         this.connectionButton = new System.Windows.Forms.Button();
         this.disconnectButton = new System.Windows.Forms.Button();
         this.connectionStatusLabel = new System.Windows.Forms.Label();
         this.receivedDataTextBox = new System.Windows.Forms.TextBox();
         this.receivedDataLabel = new System.Windows.Forms.Label();
         this.transmittedDataLabel = new System.Windows.Forms.Label();
         this.transmittedDataTextBox = new System.Windows.Forms.TextBox();
         this.clearReceivedButton = new System.Windows.Forms.Button();
         this.clearTransmitButton = new System.Windows.Forms.Button();
         this.sendTextBox = new System.Windows.Forms.TextBox();
         this.sendButton = new System.Windows.Forms.Button();
         this.portLabel = new System.Windows.Forms.Label();
         this.portComboBox = new System.Windows.Forms.ComboBox();
         this.startNewFileButton = new System.Windows.Forms.Button();
         this.selectLogFilePathButton = new System.Windows.Forms.Button();
         this.pathLabel = new System.Windows.Forms.Label();
         this.currentPathLabel = new System.Windows.Forms.Label();
         this.filenameLabel = new System.Windows.Forms.Label();
         this.closeFileButton = new System.Windows.Forms.Button();
         this.matlabButton = new System.Windows.Forms.Button();
         this.SuspendLayout();
         // 
         // connectionButton
         // 
         this.connectionButton.Location = new System.Drawing.Point(1147, 83);
         this.connectionButton.Name = "connectionButton";
         this.connectionButton.Size = new System.Drawing.Size(75, 23);
         this.connectionButton.TabIndex = 0;
         this.connectionButton.Text = "Connect";
         this.connectionButton.UseVisualStyleBackColor = true;
         this.connectionButton.Click += new System.EventHandler(this.connectionButton_Click);
         // 
         // disconnectButton
         // 
         this.disconnectButton.Location = new System.Drawing.Point(1147, 112);
         this.disconnectButton.Name = "disconnectButton";
         this.disconnectButton.Size = new System.Drawing.Size(75, 23);
         this.disconnectButton.TabIndex = 1;
         this.disconnectButton.Text = "Disconnect";
         this.disconnectButton.UseVisualStyleBackColor = true;
         this.disconnectButton.Click += new System.EventHandler(this.disconnectButton_Click);
         // 
         // connectionStatusLabel
         // 
         this.connectionStatusLabel.AutoSize = true;
         this.connectionStatusLabel.Location = new System.Drawing.Point(1091, 54);
         this.connectionStatusLabel.Name = "connectionStatusLabel";
         this.connectionStatusLabel.Size = new System.Drawing.Size(40, 13);
         this.connectionStatusLabel.TabIndex = 2;
         this.connectionStatusLabel.Text = "Status:";
         // 
         // receivedDataTextBox
         // 
         this.receivedDataTextBox.Location = new System.Drawing.Point(13, 41);
         this.receivedDataTextBox.MaxLength = 2147483646;
         this.receivedDataTextBox.Multiline = true;
         this.receivedDataTextBox.Name = "receivedDataTextBox";
         this.receivedDataTextBox.ReadOnly = true;
         this.receivedDataTextBox.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
         this.receivedDataTextBox.Size = new System.Drawing.Size(562, 608);
         this.receivedDataTextBox.TabIndex = 3;
         // 
         // receivedDataLabel
         // 
         this.receivedDataLabel.AutoSize = true;
         this.receivedDataLabel.Location = new System.Drawing.Point(13, 13);
         this.receivedDataLabel.Name = "receivedDataLabel";
         this.receivedDataLabel.Size = new System.Drawing.Size(79, 13);
         this.receivedDataLabel.TabIndex = 4;
         this.receivedDataLabel.Text = "Received Data";
         // 
         // transmittedDataLabel
         // 
         this.transmittedDataLabel.AutoSize = true;
         this.transmittedDataLabel.Location = new System.Drawing.Point(608, 12);
         this.transmittedDataLabel.Name = "transmittedDataLabel";
         this.transmittedDataLabel.Size = new System.Drawing.Size(88, 13);
         this.transmittedDataLabel.TabIndex = 5;
         this.transmittedDataLabel.Text = "Transmitted Data";
         // 
         // transmittedDataTextBox
         // 
         this.transmittedDataTextBox.Location = new System.Drawing.Point(611, 41);
         this.transmittedDataTextBox.MaxLength = 2147483646;
         this.transmittedDataTextBox.Multiline = true;
         this.transmittedDataTextBox.Name = "transmittedDataTextBox";
         this.transmittedDataTextBox.ReadOnly = true;
         this.transmittedDataTextBox.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
         this.transmittedDataTextBox.Size = new System.Drawing.Size(470, 235);
         this.transmittedDataTextBox.TabIndex = 6;
         // 
         // clearReceivedButton
         // 
         this.clearReceivedButton.Location = new System.Drawing.Point(457, 8);
         this.clearReceivedButton.Name = "clearReceivedButton";
         this.clearReceivedButton.Size = new System.Drawing.Size(90, 23);
         this.clearReceivedButton.TabIndex = 7;
         this.clearReceivedButton.Text = "Clear Received";
         this.clearReceivedButton.UseVisualStyleBackColor = true;
         this.clearReceivedButton.Click += new System.EventHandler(this.clearReceivedButton_Click);
         // 
         // clearTransmitButton
         // 
         this.clearTransmitButton.Location = new System.Drawing.Point(952, 8);
         this.clearTransmitButton.Name = "clearTransmitButton";
         this.clearTransmitButton.Size = new System.Drawing.Size(99, 23);
         this.clearTransmitButton.TabIndex = 8;
         this.clearTransmitButton.Text = "Clear Transmitted";
         this.clearTransmitButton.UseVisualStyleBackColor = true;
         this.clearTransmitButton.Click += new System.EventHandler(this.clearTransmitButton_Click);
         // 
         // sendTextBox
         // 
         this.sendTextBox.Location = new System.Drawing.Point(611, 282);
         this.sendTextBox.Name = "sendTextBox";
         this.sendTextBox.Size = new System.Drawing.Size(424, 20);
         this.sendTextBox.TabIndex = 9;
         this.sendTextBox.Enter += new System.EventHandler(this.sendTextBox_Enter);
         this.sendTextBox.Leave += new System.EventHandler(this.sendTextBox_Leave);
         // 
         // sendButton
         // 
         this.sendButton.Location = new System.Drawing.Point(1041, 280);
         this.sendButton.Name = "sendButton";
         this.sendButton.Size = new System.Drawing.Size(40, 23);
         this.sendButton.TabIndex = 10;
         this.sendButton.Text = "Send";
         this.sendButton.UseVisualStyleBackColor = true;
         this.sendButton.Click += new System.EventHandler(this.sendButton_Click);
         // 
         // portLabel
         // 
         this.portLabel.AutoSize = true;
         this.portLabel.Location = new System.Drawing.Point(1091, 17);
         this.portLabel.Name = "portLabel";
         this.portLabel.Size = new System.Drawing.Size(29, 13);
         this.portLabel.TabIndex = 11;
         this.portLabel.Text = "Port:";
         // 
         // portComboBox
         // 
         this.portComboBox.FormattingEnabled = true;
         this.portComboBox.Location = new System.Drawing.Point(1134, 17);
         this.portComboBox.Name = "portComboBox";
         this.portComboBox.Size = new System.Drawing.Size(88, 21);
         this.portComboBox.Sorted = true;
         this.portComboBox.TabIndex = 12;
         // 
         // startNewFileButton
         // 
         this.startNewFileButton.Location = new System.Drawing.Point(625, 337);
         this.startNewFileButton.Name = "startNewFileButton";
         this.startNewFileButton.Size = new System.Drawing.Size(107, 23);
         this.startNewFileButton.TabIndex = 13;
         this.startNewFileButton.Text = "Start New Text File";
         this.startNewFileButton.UseVisualStyleBackColor = true;
         this.startNewFileButton.Click += new System.EventHandler(this.startNewFileButton_Click);
         // 
         // selectLogFilePathButton
         // 
         this.selectLogFilePathButton.Location = new System.Drawing.Point(611, 308);
         this.selectLogFilePathButton.Name = "selectLogFilePathButton";
         this.selectLogFilePathButton.Size = new System.Drawing.Size(131, 23);
         this.selectLogFilePathButton.TabIndex = 14;
         this.selectLogFilePathButton.Text = "Select Log File Location";
         this.selectLogFilePathButton.UseVisualStyleBackColor = true;
         this.selectLogFilePathButton.Click += new System.EventHandler(this.selectLogFilePathButton_Click);
         // 
         // pathLabel
         // 
         this.pathLabel.AutoSize = true;
         this.pathLabel.Location = new System.Drawing.Point(748, 313);
         this.pathLabel.Name = "pathLabel";
         this.pathLabel.Size = new System.Drawing.Size(90, 13);
         this.pathLabel.TabIndex = 15;
         this.pathLabel.Text = "Selected Loction:";
         // 
         // currentPathLabel
         // 
         this.currentPathLabel.AutoSize = true;
         this.currentPathLabel.Location = new System.Drawing.Point(844, 313);
         this.currentPathLabel.Name = "currentPathLabel";
         this.currentPathLabel.Size = new System.Drawing.Size(0, 13);
         this.currentPathLabel.TabIndex = 16;
         // 
         // filenameLabel
         // 
         this.filenameLabel.AutoSize = true;
         this.filenameLabel.Location = new System.Drawing.Point(748, 342);
         this.filenameLabel.Name = "filenameLabel";
         this.filenameLabel.Size = new System.Drawing.Size(0, 13);
         this.filenameLabel.TabIndex = 17;
         // 
         // closeFileButton
         // 
         this.closeFileButton.Location = new System.Drawing.Point(634, 366);
         this.closeFileButton.Name = "closeFileButton";
         this.closeFileButton.Size = new System.Drawing.Size(89, 23);
         this.closeFileButton.TabIndex = 18;
         this.closeFileButton.Text = "Close Open File";
         this.closeFileButton.UseVisualStyleBackColor = true;
         this.closeFileButton.Click += new System.EventHandler(this.closeFileButton_Click);
         // 
         // matlabButton
         // 
         this.matlabButton.Enabled = false;
         this.matlabButton.Location = new System.Drawing.Point(640, 395);
         this.matlabButton.Name = "matlabButton";
         this.matlabButton.Size = new System.Drawing.Size(75, 23);
         this.matlabButton.TabIndex = 19;
         this.matlabButton.Text = "Matlab";
         this.matlabButton.UseVisualStyleBackColor = true;
         this.matlabButton.Click += new System.EventHandler(this.matlabButton_Click);
         // 
         // Form1
         // 
         this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
         this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
         this.ClientSize = new System.Drawing.Size(1234, 661);
         this.Controls.Add(this.matlabButton);
         this.Controls.Add(this.closeFileButton);
         this.Controls.Add(this.filenameLabel);
         this.Controls.Add(this.currentPathLabel);
         this.Controls.Add(this.pathLabel);
         this.Controls.Add(this.selectLogFilePathButton);
         this.Controls.Add(this.startNewFileButton);
         this.Controls.Add(this.portComboBox);
         this.Controls.Add(this.portLabel);
         this.Controls.Add(this.sendButton);
         this.Controls.Add(this.sendTextBox);
         this.Controls.Add(this.clearTransmitButton);
         this.Controls.Add(this.clearReceivedButton);
         this.Controls.Add(this.transmittedDataTextBox);
         this.Controls.Add(this.transmittedDataLabel);
         this.Controls.Add(this.receivedDataLabel);
         this.Controls.Add(this.receivedDataTextBox);
         this.Controls.Add(this.connectionStatusLabel);
         this.Controls.Add(this.disconnectButton);
         this.Controls.Add(this.connectionButton);
         this.Name = "Form1";
         this.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen;
         this.Text = "Quadrotor Control";
         this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.Form1_FormClosing);
         this.Paint += new System.Windows.Forms.PaintEventHandler(this.Form1_Paint);
         this.ResumeLayout(false);
         this.PerformLayout();

      }

      #endregion

      private System.Windows.Forms.Button connectionButton;
      private System.Windows.Forms.Button disconnectButton;
      private System.Windows.Forms.Label connectionStatusLabel;
      private System.Windows.Forms.TextBox receivedDataTextBox;
      private System.Windows.Forms.Label receivedDataLabel;
      private System.Windows.Forms.Label transmittedDataLabel;
      private System.Windows.Forms.TextBox transmittedDataTextBox;
      private System.Windows.Forms.Button clearReceivedButton;
      private System.Windows.Forms.Button clearTransmitButton;
      private System.Windows.Forms.TextBox sendTextBox;
      private System.Windows.Forms.Button sendButton;
      private System.Windows.Forms.Label portLabel;
      private System.Windows.Forms.ComboBox portComboBox;
      private System.Windows.Forms.Button startNewFileButton;
      private System.Windows.Forms.Button selectLogFilePathButton;
      private System.Windows.Forms.Label pathLabel;
      private System.Windows.Forms.Label currentPathLabel;
      private System.Windows.Forms.Label filenameLabel;
      private System.Windows.Forms.Button closeFileButton;
      private System.Windows.Forms.Button matlabButton;

   }
}

