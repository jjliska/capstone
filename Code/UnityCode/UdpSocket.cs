/*
  AME 485/486 - Capstone
  Team: Reflection
  Members:   Joshua Liska - Programming/Engineering/Fusion360 Modeling/Building
            Ivan Mendoza - Programming/Building
            Jack Carroll - Sound Designer/Unity Programming
            Albert Bang - Visual Artist/Unity Programming/3D Modeling

  References: Two-way communication between Python 3 and Unity (C#) - Y. T. Elashry @ https://github.com/Siliconifier/Python-Unity-Socket-Communication.git

  Use Case: A user will step infront of the camera and the camera will then being to track that user.
  Every (delayPeriod) the emotion of the user will be taken and passed to a unity program that will also use facial reference data
  to calculate where the user is and look at them. This facial data will also be used to drive several servos to a corrected position
  and follow their movements accordingly.
*/

using UnityEngine;
using System.Collections;
using System;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using Random = UnityEngine.Random;

public class UdpSocket : MonoBehaviour{
  [HideInInspector] public bool isTxStarted = false;

  [SerializeField] string IP = "127.0.0.1"; // local host
  [SerializeField] int rxPort = 8000; // port to receive data from Python on
  [SerializeField] int txPort = 8001; // port to send data to Python on

  // Create necessary UdpClient objects
  UdpClient client;
  IPEndPoint remoteEndPoint;
  Thread receiveThread; // Receiving Thread

  Vector3 TargetPosition;

  float x = 0.0f;
  float y = 0.0f;
  float rotation = 0.0f;
  string emotion = "";
  string previousEmotion = "";
  int track = 0;
  int previousTrack = 0;

  float zConst = -60.0f;

  public GameObject humanFace;

  public GameObject humanFaceParent;
  public GameObject gasMaskParent;

  public GameObject audioRotation;

  public AudioSource musicPlayer1;
  public AudioSource musicPlayer2;
  public AudioSource musicPlayer3;

  SkinnedMeshRenderer skinnedMeshRenderer;
  Mesh skinnedMesh;
  public const float blendFaceStop = 60f;
  float blendIdle = 0.0f;
  float blendEmotion = 0.0f;
  const float blendSpeed = 1.0f;
  const float blendSpeedEmotion = 0.25f;
  const float volumeSpeed = ((100f/blendFaceStop)*blendSpeedEmotion)/100f;
  float volume = 0.0f;
  int waitTime = 0;
  bool idleFinished = false;
  bool flipflop = false;
  int counter = 0;

  void Awake(){
    // Create remote endpoint (to Matlab)
    remoteEndPoint = new IPEndPoint(IPAddress.Parse(IP), txPort);

    // Create local client
    client = new UdpClient(rxPort);

    // local endpoint define (where messages are received)
    // Create a new thread for reception of incoming messages
    receiveThread = new Thread(new ThreadStart(ReceiveData));
    receiveThread.IsBackground = true;
    receiveThread.Start();

    // Initialize (seen in comments window)
    print("UDP Comms Initialised");

    skinnedMeshRenderer = humanFace.GetComponent<SkinnedMeshRenderer> ();
    skinnedMesh = humanFace.GetComponent<SkinnedMeshRenderer> ().sharedMesh;
  }

  // Starting with a random number so the blink doesnt immediately happen on startup
  void Start(){
    waitTime = Random.Range(300,1200);
  }

  // Updates the target to move the face in real time no smoothing is necessary as the webcam runs fast enough for the machine to not need smoothing
  void Update(){
    TargetPosition = new Vector3(-1*x/25.0f, y/25.0f, zConst);
    transform.position = TargetPosition;
    audioRotation.transform.Rotate(0,rotation,0);

    idleHandler();
    //Smooth ramp from 60 to 0 and back to 60
    emotionHandler();
    //Smooth ramp from 100 to 0 and back to 100
    musicHandler();
  }

  // Handles the animation of the character, if a new emotion is input it slowly brings it to zero then starts the next animation
  private void emotionHandler(){
    if(previousEmotion == ""){
      if(blendEmotion < blendFaceStop){
        blendEmotion += blendSpeedEmotion;
        emotionCase(emotion);
      }
      else{
        previousEmotion = emotion;
      }
    }
    else{
      if(previousEmotion != emotion){
        if(blendEmotion > 0f){
          blendEmotion -= blendSpeedEmotion;
          emotionCase(previousEmotion);
        }
        else{
          previousEmotion = emotion;
        }
      }
      else{
        if(blendEmotion < blendFaceStop){
          blendEmotion += blendSpeedEmotion;
          emotionCase(emotion);
        }
      }
    }
  }

  private void musicHandler(){
    int compEmot = musicComparison(emotion);
    int compPreEmot = musicComparison(previousEmotion);
    if(compPreEmot == 0){
      if(volume < 1.0f){
        volume += volumeSpeed;
        musicCase(compEmot);
      }
    }
    else{
      if(compPreEmot != compEmot){
        if(volume > 0f){
          volume -= volumeSpeed;
          musicCase(compPreEmot);
        }
      }
      else{
        if(volume < 1.0f){
          volume += volumeSpeed;
          musicCase(compEmot);
        }
      }
    }
  }

  // Change this to add more audio tracks
  private void musicCase(int input){
    if(input != 0){
      if(input == 1){
        musicPlayer1.volume = volume;
      }
      else if(input == 2){
        musicPlayer2.volume = volume;
      }
      else if(input == 3){
        musicPlayer3.volume = volume;
      }
    }
  }


  // This is made easier to access to allow for more string comparisons as there may be a change to the audio track later
  private int musicComparison(string input){
    if(input == "Happy" || input == "Surprise"){
      return 1;
    }
    else if(input == "Anger" || input == "Disgust" || input == "Fear"){
      return 2;
    }
    else if(input == "Neutral" || input == "Sad"){
      return 3;
    }
    else{
      return 0;
    }
  }

  private void emotionCase(string input){
    //Determining which object is active/visible in the scene
    if(input == "Fear"){
      humanFaceParent.active = false;
      gasMaskParent.active = true;
    }
    else{
      humanFaceParent.active = true;
      gasMaskParent.active = false;
    }

    //Emotion Case to identify what needs to happen visually to interact with the user
    switch(input){
      case "Anger":
        skinnedMeshRenderer.SetBlendShapeWeight(0, blendEmotion);
        break;
      case "Disgust":
        skinnedMeshRenderer.SetBlendShapeWeight(3, blendEmotion);
        skinnedMeshRenderer.SetBlendShapeWeight(3, blendEmotion);
        break;
      case "Fear":
        gasMaskParent.active = true;
        break;
      case "Happy":
        skinnedMeshRenderer.SetBlendShapeWeight(2, blendEmotion);
        break;
      case "Sad":
        skinnedMeshRenderer.SetBlendShapeWeight(3, blendEmotion);
        break;
      case "Surprise":
        skinnedMeshRenderer.SetBlendShapeWeight(1, blendEmotion);
        break;
      case "Neutral":
        //Do nothing its a neutral expression
        break;
      default:
        humanFaceParent.active = true;
        break;
    }
  }

  // Handles the idle animation blinking randomly every .3 to 1.2s
  private void idleHandler(){
    if(!idleFinished){
      if(!flipflop){
        if (blendIdle < 60f){
          skinnedMeshRenderer.SetBlendShapeWeight(4, blendIdle);
          skinnedMeshRenderer.SetBlendShapeWeight(5, blendIdle);
          blendIdle += blendSpeed;
        }
        else{
          flipflop = true;
        }
      }
      else{
        if (blendIdle > 0f){
          skinnedMeshRenderer.SetBlendShapeWeight(4, blendIdle);
          skinnedMeshRenderer.SetBlendShapeWeight(5, blendIdle);
          blendIdle -= blendSpeed;
        }
        else{
          flipflop = false;
          waitTime = Random.Range(300,1200);
          idleFinished = true;
          Debug.Log("State change");
        }
      }
    }
    else{
      counter+=1;
      if(counter > waitTime){
        counter=0;
        idleFinished = false;
      }
    }
  }

  // Receive data, update packets received
  private void ReceiveData(){
    while (true){
      try{
        IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, 0);
        byte[] data = client.Receive(ref anyIP);
        string text = Encoding.UTF8.GetString(data);
        ProcessInput(text);
      }
      catch (Exception err){
        Debug.Log(err.ToString());
      }
    }
  }

  //Void for processing what happens with FaceData, emotion, and zrot from the given input
  private void ProcessInput(string input){
    string[] split = input.Split(',');
    x = float.Parse(split[0]);
    y = float.Parse(split[1]);
    // currently unused as the istage requires a license to access 7.1 surround sound
    rotation = float.Parse(split[2]);
    emotion = split[3];
    Debug.Log(x+","+y+","+rotation+","+emotion);

    if (!isTxStarted){
        isTxStarted = true;
    }
  }

  //Prevent crashes - close clients and threads properly!
  void OnDisable(){
    if (receiveThread != null)
      receiveThread.Abort();

    client.Close();
  }
}
