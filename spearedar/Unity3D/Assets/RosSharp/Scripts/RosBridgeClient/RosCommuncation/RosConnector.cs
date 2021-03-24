/*
© Siemens AG, 2017-2019
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

<http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using System;
using System.Threading;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class RosConnector : MonoBehaviour
    {
        public int SecondsTimeout = 10;

        public RosSocket RosSocket { get; private set; }
        public enum Protocols { WebSocketSharp, WebSocketNET, WebSocketUWP };
        public RosSocket.SerializerEnum Serializer;
        public Protocols Protocol;
        #if !WINDOWS_UWP
        public RosBridgeClient.Protocols.Protocol protocol;
        #endif
        public string RosBridgeServerUrl = "ws://192.168.0.1:9090";

        public ManualResetEvent IsConnected { get; private set; }

        public virtual void Awake()
        {
#if WINDOWS_UWP
            ConnectAndWait();
#else
            IsConnected = new ManualResetEvent(false);
            new Thread(ConnectAndWait).Start();
#endif
        }

        protected void ConnectAndWait()
        {
#if WINDOWS_UWP
            RosSocket = ConnectToRos(Protocol, RosBridgeServerUrl, OnConnected, OnClosed, Serializer);
#else
            RosSocket = ConnectToRos(protocol, RosBridgeServerUrl, OnConnected, OnClosed, Serializer);

            if (!IsConnected.WaitOne(SecondsTimeout * 1000))
                Debug.LogWarning("Failed to connect to RosBridge at: " + RosBridgeServerUrl);
#endif
        }

        public static RosSocket ConnectToRos(Protocols protocolType, string serverUrl, EventHandler onConnected = null, EventHandler onClosed = null, RosSocket.SerializerEnum serializer = RosSocket.SerializerEnum.JSON)
        {
            RosBridgeClient.Protocols.IProtocol protocol = GetProtocol(protocolType, serverUrl);
            protocol.OnConnected += onConnected;
            protocol.OnClosed += onClosed;
            return new RosSocket(protocol,serializer);
        }

        private static RosBridgeClient.Protocols.IProtocol GetProtocol(Protocols protocol, string rosBridgeServerUrl)
        {
#if WINDOWS_UWP
            return new RosBridgeClient.Protocols.WebSocketUWPProtocol(rosBridgeServerUrl);
#else
            return null;
#endif
        }
#if !WINDOWS_UWP
        public static RosSocket ConnectToRos(RosBridgeClient.Protocols.Protocol protocolType, string serverUrl, EventHandler onConnected = null, EventHandler onClosed = null, RosSocket.SerializerEnum serializer = RosSocket.SerializerEnum.JSON)
        {
            RosBridgeClient.Protocols.IProtocol protocol = RosBridgeClient.Protocols.ProtocolInitializer.GetProtocol(protocolType, serverUrl);
            protocol.OnConnected += onConnected;
            protocol.OnClosed += onClosed;
            return new RosSocket(protocol, serializer);
            
        }
#endif
        private void OnApplicationQuit()
        {
            RosSocket.Close();
        }

        private void OnConnected(object sender, EventArgs e)
        {
#if !WINDOWS_UWP
            IsConnected.Set();
#endif
            Debug.Log("Connected to RosBridge: " + RosBridgeServerUrl);
        }

        private void OnClosed(object sender, EventArgs e)
        {
#if !WINDOWS_UWP
            IsConnected.Reset();
#endif
            Debug.Log("Disconnected from RosBridge: " + RosBridgeServerUrl);
        }
    }
}
