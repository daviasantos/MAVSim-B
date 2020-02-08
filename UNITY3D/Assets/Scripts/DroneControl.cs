using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Threading;
using System.Net;
using System.Net.Sockets;
using UnityEngine;

public class DroneControl : MonoBehaviour
{
    Vector3 dronePos, droneAtt;

    static Mutex mut;
    public Rigidbody body;
    private bool runServer = false;

    public float Power;


    Task serverListener;
    TcpListener server;

    // Start is called before the first frame update 
    void Awake()
    {
        Vector3 origin = new Vector3(0, 0, 0); 
        dronePos = origin;
        droneAtt = origin;

        mut = new Mutex();
        runServer = true;
        serverListener = new Task(ReceiveData); 
        serverListener.Start();
        
    }


    
    void FixedUpdate()
    {
        mut.WaitOne();
        Sound();
        transform.position = dronePos;
        transform.localEulerAngles = droneAtt;
        mut.ReleaseMutex();

    }

    public AudioSource sound;

    void Sound() 
    {
        if (Power == 1f){

            sound.volume = 1;
            body.useGravity = false;
        }else {

            sound.volume = 0;
            body.useGravity = true;
        }

    }

    private void OnDestroy()
    {
        runServer = false;
        try
        {
            server.Stop();
        }
        finally
        {
            print("Server was not working!!!!");
        }
    }

    //  Function to receive TCP data.
    public void ReceiveData()
    {

        try
        {
            int port = 55001;
            //  Setup UDP client.
            server = new TcpListener(IPAddress.Any, port);
            server.Start();



            print("TCP Server Activated!");
            print("Listen: " + port.ToString());

            TcpClient client = server.AcceptTcpClient();
            NetworkStream stream = client.GetStream();

            print("Connection established!!!");

            float[] data = new float[9];
            byte[] rawData = new byte[4 * data.Length];
            int n;

            byte[] outData = new byte[sizeof(int)];
            int[] cont = { 0 };
            

            while (runServer)
            {
                try
                {
                    n = stream.Read(rawData, 0, rawData.Length);

                    //if (n > 0 && n < rawData.Length)
                    //    continue;
                    Buffer.BlockCopy(rawData, 0, data, 0, n);
                    if (n == 28)
                    {
                        Vector3 pos = new Vector3(0, 0, 0);
                        Vector3 att = new Vector3(0, 0, 0);

                        Power = data[6];

                        pos.x = -data[1];
                        pos.y = data[2];
                        pos.z = data[0];

                        att.x = -data[4];
                        att.y = -data[5];
                        att.z = data[3];

                        mut.WaitOne();

                        dronePos = pos;
                        droneAtt = att;

                        mut.ReleaseMutex();

                        //cont[0]++;

                        //Buffer.BlockCopy(cont, 0, outData, 0, sizeof(int));

                        //stream.Write(outData, 0, sizeof(int));
                        stream.Write(rawData, 0, rawData.Length);
                    }
                    else
                    {
                        print(">>>>>> " + n.ToString());

                        for (int i = 0; i < n; i++)
                            print(data[i]);
                    }
                        
                }
                catch (Exception err)
                {
                    print(">> TCP Server Error: " + err.Message);
                }
            }
        }
        finally
        {
            print("TCP Server not working!!!");
        }
    }
}
