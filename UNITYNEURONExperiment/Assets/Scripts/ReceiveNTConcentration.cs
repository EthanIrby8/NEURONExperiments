using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Text;
using System.Threading;
using System;
using SocketIOClient;


public class ReceiveNTConcentration : MonoBehaviour
{
    public GameObject basalGangliaObject;
    public SocketIOUnity socket;
    public int port = 25001;

    // Start is called before the first frame update
    async void Start()
    {
        var uri = new Uri("http://localhost:25001");
        socket = new SocketIOUnity(uri, new SocketIOOptions
        {
            Query = new Dictionary<string, string>
                {
                    {"token", "UNITY" }
                }
            ,
            Transport = SocketIOClient.Transport.TransportProtocol.WebSocket
        });
        socket.OnConnected += (sender, e) =>
        {
            Debug.Log("socket.OnConnected");
        };
        StartServer();
        socket.Connect();
        await socket.ConnectAsync();
    }

    void StartServer()  
    {
        socket.On("nt_concentration", (response) => 
        {
            float concentrationResponse = response.GetValue().GetSingle();
            Debug.Log("Received data: " + response.GetValue().ToString());
            UnityThread.executeInUpdate(() => 
            {
                UpdateColor(concentrationResponse);
            });
        });
    }

    void UpdateColor(float concentrationValue)
    {
        Renderer renderer = basalGangliaObject.GetComponentInChildren<Renderer>();
        Material m = renderer.material;
        if (renderer == null)
        {
            Debug.Log("Basal Ganglia GameObject does not have a renderer component");
        }

        if (concentrationValue > 0f && concentrationValue < 0.5f)
        {
            m.color = Color.red;
            Debug.Log("Changed color to red");
        }
        else if (concentrationValue > 0.5f && concentrationValue < 0.8f)
        {
            m.color = Color.green;
            Debug.Log("Changed color to green");
        }
        else if (concentrationValue > 0.8f && concentrationValue <= 1f)
        {
            m.color = Color.blue; 
            Debug.Log("Changed color to blue");
        }
        else 
        {
            m.color = Color.white; 
            Debug.Log("Changed color to white");
        }
    }

    async void OnApplicationQuit()
    {
        Debug.Log("Disconnecting Socket");
        if (socket != null && socket.Connected) 
        {
            await socket.DisconnectAsync();
        }
        socket?.Dispose();
    }


    // Update is called once per frame
    void Update()
    {
        
    }
}
