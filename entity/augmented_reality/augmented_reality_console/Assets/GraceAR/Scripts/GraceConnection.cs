using System;
using System.Collections.Concurrent;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

namespace Aiva.GraceAR
{
    // Uses the same ROS chat node and memory as the browser; no LLM credentials on the phone.
    public sealed class GraceConnection : MonoBehaviour
    {
        [Serializable] private class Envelope { public string op; public string topic; public string type; public Payload msg; }
        [Serializable] private class Payload { public string data; }
        [Serializable] private class Prompt { public string text; public string identifier; }
        [Serializable] private class Reply { public string type; public string content; public string radio_id; }
        private readonly ConcurrentQueue<string> _inbox = new();
        private ClientWebSocket _socket;
        private CancellationTokenSource _cancel;
        private string _request;
        private float _deadline;
        public string Response { get; private set; } = "Connect to AIVA to talk with Grace.";
        public bool Busy { get; private set; }
        public bool Connected => _socket?.State == WebSocketState.Open;

        public async void Connect(string gateway)
        {
            if (Busy || Connected) return;
            Busy = true; _request = null;
            try
            {
                var url = new Uri(gateway.Trim());
                if (url.Scheme != "http" && url.Scheme != "https") throw new ArgumentException("Use an http:// or https:// WebUI address.");
                _cancel?.Cancel(); _socket?.Dispose(); _cancel?.Dispose();
                _cancel = new CancellationTokenSource(); _socket = new ClientWebSocket();
                var socket = _socket;
                var token = _cancel.Token;
                var builder = new UriBuilder(url) { Scheme = url.Scheme == "https" ? "wss" : "ws", Path = "/rosbridge", Query = "", Fragment = "" };
                using (var timeout = CancellationTokenSource.CreateLinkedTokenSource(token))
                {
                    timeout.CancelAfter(TimeSpan.FromSeconds(10));
                    await socket.ConnectAsync(builder.Uri, timeout.Token);
                }
                await Send(new Envelope { op = "subscribe", topic = "/tms/text_response", type = "std_msgs/String" });
                await Send(new Envelope { op = "advertise", topic = "/aiva/chat_input", type = "std_msgs/String" });
                Response = "Connected. What would you like to say?";
                _ = Receive(socket, token);
            }
            catch (Exception error) { Response = "Connection failed: " + error.Message; _socket?.Abort(); }
            finally { Busy = false; }
        }

        public async void Ask(string text)
        {
            if (!Connected || Busy || string.IsNullOrWhiteSpace(text)) return;
            Busy = true; _request = Guid.NewGuid().ToString("N"); _deadline = Time.realtimeSinceStartup + 150f;
            Response = "Grace is thinking…";
            try
            {
                await Send(new Envelope { op = "publish", topic = "/aiva/chat_input", msg = new Payload {
                    data = JsonUtility.ToJson(new Prompt { text = text.Substring(0, Math.Min(text.Length, 4000)), identifier = _request }) } });
            }
            catch (Exception error) { Response = error.Message; Busy = false; _request = null; }
        }

        private Task Send(Envelope message)
        {
            var bytes = Encoding.UTF8.GetBytes(JsonUtility.ToJson(message));
            return _socket.SendAsync(new ArraySegment<byte>(bytes), WebSocketMessageType.Text, true, _cancel.Token);
        }

        private async Task Receive(ClientWebSocket socket, CancellationToken token)
        {
            var buffer = new byte[8192];
            try
            {
                while (!token.IsCancellationRequested && socket.State == WebSocketState.Open)
                {
                    using var stream = new System.IO.MemoryStream();
                    WebSocketReceiveResult part;
                    do
                    {
                        part = await socket.ReceiveAsync(new ArraySegment<byte>(buffer), token);
                        if (part.MessageType == WebSocketMessageType.Close) { _inbox.Enqueue("Connection closed. Connect again to resume."); return; }
                        stream.Write(buffer, 0, part.Count);
                        if (stream.Length > 1024 * 1024) throw new InvalidOperationException("ROS response exceeds 1 MiB.");
                    } while (!part.EndOfMessage);
                    _inbox.Enqueue(Encoding.UTF8.GetString(stream.ToArray()));
                }
            }
            catch (Exception error) { if (!token.IsCancellationRequested) _inbox.Enqueue("Connection lost: " + error.Message); }
        }

        private void Update()
        {
            while (_inbox.TryDequeue(out var raw))
            {
                if (!raw.StartsWith("{")) { Response = raw; Busy = false; continue; }
                try
                {
                    var envelope = JsonUtility.FromJson<Envelope>(raw);
                    if (envelope.topic != "/tms/text_response" || envelope.msg == null) continue;
                    var reply = JsonUtility.FromJson<Reply>(envelope.msg.data);
                    if (_request == null) continue;
                    if (reply == null || reply.radio_id != _request) continue;
                    if (reply.type == "start") Response = "";
                    if (reply.type == "delta") Response += reply.content;
                    if (reply.type == "error") Response = reply.content;
                    if (reply.type == "done" || reply.type == "error") { Busy = false; _request = null; }
                }
                catch (ArgumentException) { /* Ignore unrelated ROS traffic. */ }
            }
            if (Busy && _request != null && Time.realtimeSinceStartup > _deadline)
            { Busy = false; _request = null; Response = "Grace did not respond in time. Try again."; }
        }

        public void Disconnect()
        {
            _request = null; Busy = false;
            _cancel?.Cancel(); _socket?.Abort(); _socket?.Dispose(); _cancel?.Dispose();
            _socket = null; _cancel = null;
            while (_inbox.TryDequeue(out _)) { }
            Response = "Disconnected. You can change the AIVA gateway and connect again.";
        }

        private void OnDestroy() { Disconnect(); }
    }
}
