using Microsoft.Azure.Kinect.BodyTracking;
using Microsoft.Azure.Kinect.Sensor;
using System;
using System.IO;
using System.Linq;
using System.Text;

namespace Csharp_3d_viewer
{
    class Program
    {
        static void Main()
        {
            using (var visualizerData = new VisualizerData())
            {
                var renderer = new Renderer(visualizerData);

                renderer.StartVisualizationThread();

                // Open device.
                using (Device device = Device.Open())
                {
                    var fileName = $"bt-dump-{DateTime.Now:yyyy-MM-dd_hh-mm-ss}.btd";
                    using (BinaryWriter file = new BinaryWriter(new FileStream(fileName, FileMode.Create, FileAccess.Write)))
                    {
                        file.Write(Encoding.ASCII.GetBytes("BT-DUMP\nV: 1\n"));
                        WriteVersion(file, device.Version);
                        file.Write(Encoding.ASCII.GetBytes($"device-sn: {device.SerialNum}\n"));

                        device.StartCameras(new DeviceConfiguration()
                        {
                            CameraFPS = FPS.FPS30,
                            ColorResolution = ColorResolution.Off,
                            DepthMode = DepthMode.NFOV_Unbinned,
                            WiredSyncMode = WiredSyncMode.Standalone,
                        });

                        file.Write(Encoding.ASCII.GetBytes($"device-color-resolution: {device.CurrentColorResolution}\n"));
                        file.Write(Encoding.ASCII.GetBytes($"device-depth-mode: {device.CurrentDepthMode}\n"));

                        var deviceCalibration = device.GetCalibration();
                        PointCloud.ComputePointCloudCache(deviceCalibration);

                        WriteCalibration(file, "depth", deviceCalibration.DepthCameraCalibration);
                        WriteCalibration(file, "color", deviceCalibration.ColorCameraCalibration);

                        using (Tracker tracker = Tracker.Create(deviceCalibration, new TrackerConfiguration() { ProcessingMode = TrackerProcessingMode.Gpu, SensorOrientation = SensorOrientation.Default }))
                        {
                            file.Write(Encoding.ASCII.GetBytes($"joint-count: {Skeleton.JointCount}\n"));
                            file.Write(Encoding.ASCII.GetBytes("data:\n"));

                            while (renderer.IsActive)
                            {
                                using (Capture sensorCapture = device.GetCapture())
                                {
                                    // Queue latest frame from the sensor.
                                    tracker.EnqueueCapture(sensorCapture);
                                }

                                // Try getting latest tracker frame.
                                using (Frame frame = tracker.PopResult(TimeSpan.Zero, throwOnTimeout: false))
                                {
                                    if (frame != null)
                                    {
                                        // Save to recording file.
                                        file.Write(frame.DeviceTimestamp.Ticks);
                                        file.Write(frame.NumberOfBodies);
                                        for (uint i = 0; i < frame.NumberOfBodies; i++)
                                        {
                                            var person = frame.GetBodyId(i);
                                            file.Write(person);
                                            var s = frame.GetBodySkeleton(i);
                                            for (uint j = 0; j < Skeleton.JointCount; j++)
                                            {
                                                var joint = s.GetJoint((JointId)j);
                                                var c = joint.ConfidenceLevel;
                                                var p = joint.Position;
                                                var r = joint.Quaternion;
                                                file.Write((byte)c);
                                                file.Write(p.X); file.Write(p.Y); file.Write(p.Z);
                                                file.Write(r.X); file.Write(r.Y); file.Write(r.Z); file.Write(r.W);
                                            }
                                        }

                                        // Save this frame for visualization in Renderer.

                                        // One can access frame data here and extract e.g. tracked bodies from it for the needed purpose.
                                        // Instead, for simplicity, we transfer the frame object to the rendering background thread.
                                        // This example shows that frame popped from tracker should be disposed. Since here it is used
                                        // in a different thread, we use Reference method to prolong the lifetime of the frame object.
                                        // For reference on how to read frame data, please take a look at Renderer.NativeWindow_Render().
                                        visualizerData.Frame = frame.Reference();
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        private static void WriteVersion(BinaryWriter file, HardwareVersion v)
        {
            file.Write(Encoding.ASCII.GetBytes($"device-version-audio: {v.Audio}\n"));
            file.Write(Encoding.ASCII.GetBytes($"device-version-depth: {v.Depth}\n"));
            file.Write(Encoding.ASCII.GetBytes($"device-version-depth-sensor: {v.DepthSensor}\n"));
            file.Write(Encoding.ASCII.GetBytes($"device-version-firmware-build: {v.FirmwareBuild}\n"));
            file.Write(Encoding.ASCII.GetBytes($"device-version-firmware-signature: {v.FirmwareSignature}\n"));
            file.Write(Encoding.ASCII.GetBytes($"device-version-rgb: {v.RGB}\n"));
        }

        private static void WriteCalibration(BinaryWriter file, string prefix, CameraCalibration cal)
        {
            var rs = string.Join(" ", from f in cal.Extrinsics.Rotation select f.ToString("R"));
            file.Write(Encoding.ASCII.GetBytes($"device-{prefix}-calibration-rotation: {rs}\n"));
            var ts = string.Join(" ", from f in cal.Extrinsics.Translation select f.ToString("R"));
            file.Write(Encoding.ASCII.GetBytes($"device-{prefix}-calibration-translation: {ts}\n"));
            file.Write(Encoding.ASCII.GetBytes($"device-{prefix}-calibration-type: {cal.Intrinsics.Type}\n"));
            var ps = string.Join(" ", from f in cal.Intrinsics.Parameters select f.ToString("R"));
            file.Write(Encoding.ASCII.GetBytes($"device-{prefix}-calibration-parameters: {ps}\n"));
        }
    }
}