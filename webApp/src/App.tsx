import React, { useEffect, useRef, useState } from 'react';
import { Camera, StopCircle, Download, Shield, AlertTriangle } from 'lucide-react';

function App() {
  const videoRef = useRef<HTMLVideoElement>(null);
  const mediaRecorderRef = useRef<MediaRecorder | null>(null);
  const [isRecording, setIsRecording] = useState(false);
  const [recordedBlob, setRecordedBlob] = useState<Blob | null>(null);
  const [error, setError] = useState<string | null>(null);
  const chunks = useRef<Blob[]>([]);

  useEffect(() => {
    async function initializeCamera() {
      try {
        const stream = await navigator.mediaDevices.getUserMedia({ video: true, audio: true });
        if (videoRef.current) {
          videoRef.current.srcObject = stream;
        }
      } catch (err) {
        setError('Unable to access camera. Please ensure you have granted camera permissions.');
        console.error('Error accessing media devices:', err);
      }
    }

    initializeCamera();
  }, []);

  const startRecording = () => {
    chunks.current = [];
    const stream = videoRef.current?.srcObject as MediaStream;
    if (stream) {
      const mediaRecorder = new MediaRecorder(stream);
      mediaRecorderRef.current = mediaRecorder;
      
      mediaRecorder.ondataavailable = (event) => {
        if (event.data.size > 0) {
          chunks.current.push(event.data);
        }
      };

      mediaRecorder.onstop = () => {
        const blob = new Blob(chunks.current, { type: 'video/webm' });
        setRecordedBlob(blob);
      };

      mediaRecorder.start();
      setIsRecording(true);
    }
  };

  const stopRecording = () => {
    if (mediaRecorderRef.current) {
      mediaRecorderRef.current.stop();
      setIsRecording(false);
    }
  };

  const downloadVideo = () => {
    if (recordedBlob) {
      const url = URL.createObjectURL(recordedBlob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `firesafe-recording-${new Date().toISOString()}.webm`;
      a.click();
      URL.revokeObjectURL(url);
    }
  };

  return (
    <div className="min-h-screen bg-gradient-to-b from-gray-900 to-red-900">
      <div className="container mx-auto px-4 py-8">
        <div className="flex items-center justify-center gap-3 mb-8">
          <Shield size={32} className="text-red-500" />
          <h1 className="text-4xl font-bold text-white">FireSafe Robot</h1>
        </div>

        <div className="max-w-3xl mx-auto">
          {error ? (
            <div className="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded relative mb-4 flex items-center gap-2">
              <AlertTriangle size={20} />
              <span>{error}</span>
            </div>
          ) : null}

          <div className="bg-gray-800 rounded-lg overflow-hidden shadow-2xl">
            <video
              ref={videoRef}
              autoPlay
              playsInline
              muted
              className="w-full h-[480px] object-cover"
            />
            
            <div className="p-4 bg-gray-800 border-t border-gray-700">
              <div className="flex items-center justify-between">
                <div className="flex gap-3">
                  {!isRecording ? (
                    <button
                      onClick={startRecording}
                      className="flex items-center gap-2 bg-red-600 hover:bg-red-700 text-white px-6 py-2 rounded-md transition-colors"
                    >
                      <Camera size={20} />
                      Record
                    </button>
                  ) : (
                    <button
                      onClick={stopRecording}
                      className="flex items-center gap-2 bg-gray-600 hover:bg-gray-700 text-white px-6 py-2 rounded-md transition-colors"
                    >
                      <StopCircle size={20} />
                      Stop
                    </button>
                  )}
                </div>

                {recordedBlob && !isRecording && (
                  <button
                    onClick={downloadVideo}
                    className="flex items-center gap-2 bg-blue-600 hover:bg-blue-700 text-white px-6 py-2 rounded-md transition-colors"
                  >
                    <Download size={20} />
                    Download Recording
                  </button>
                )}
              </div>
            </div>
          </div>

          <div className="mt-6 text-center text-gray-300 text-sm">
            <p>FireSafe Robot Video Recording System</p>
            <p>Use this interface to record and monitor firefighting operations</p>
          </div>
        </div>
      </div>
    </div>
  );
}

export default App;