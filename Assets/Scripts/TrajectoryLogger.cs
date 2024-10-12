using System.Collections;
using System.Collections.Generic;
using UnityEngine;
//using Newtonsoft.Json; // Import JSON.NET from Unity Asset store (might not be needed 2022)


public class TrajectoryLogger : MonoBehaviour
{
    public string trajectory_filename = "Text/trajectory";
    private TrajectoryInfo myInfo;
    public TrajectoryInfo recordedInfo;

    public bool RecordingOn;
    public bool PlaybackOn;

    public List<Vector3> position_list = new List<Vector3>();
    public List<Vector3> rotation_list = new List<Vector3>();
    public List<float> time_list = new List<float>();

    public int current_index;


    private void Awake()
    {
        if (PlaybackOn)
        {
            var jsonTextFile = Resources.Load<TextAsset>(trajectory_filename);
            myInfo = TrajectoryInfo.CreateFromJSON(jsonTextFile.text);
        }
        recordedInfo = new TrajectoryInfo();

        current_index = 0;

    }

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void FixedUpdate()
    {
        if (PlaybackOn)
        {
            FixedUpdatePlayback();
        }
        if (RecordingOn)
        {
            FixedUpdateRecord();
        }
    }

    private void FixedUpdatePlayback()
    {
        if (Time.time <= myInfo.time_array[myInfo.time_array.Length - 1])
        {
            while (myInfo.time_array[current_index] < Time.time)
            {
                current_index++;
            }
            // avoid index out of bounds
            if (current_index > myInfo.position_array.Length - 1)
            {
                current_index = myInfo.position_array.Length - 1;
            }
            transform.position = myInfo.position_array[current_index];
            transform.rotation = Quaternion.Euler(myInfo.rotation_array[current_index]);
        }

    }

    private void FixedUpdateRecord()
    {
        time_list.Add(Time.time);
        position_list.Add(transform.position);
        rotation_list.Add(transform.rotation.eulerAngles); 
    }

    void OnApplicationQuit()
    {
        // Save data structure 
        //Debug.Log("Application ending after " + Time.time + " seconds");

        if (RecordingOn && enabled)
        {
            recordedInfo.time_array = time_list.ToArray();
            recordedInfo.position_array = position_list.ToArray();
            recordedInfo.rotation_array = rotation_list.ToArray();
            recordedInfo.file_name = "Traj" + System.DateTime.Now.ToLongTimeString().Replace(":","_").Replace(" ","") + ".json";
            //Debug.Log(System.DateTime.Now.ToLongTimeString());
            //Debug.Log(System.DateTime.Now.ToShortTimeString());


            recordedInfo.WriteDataToFile();
        }
    }
}

[System.Serializable]
public class TrajectoryInfo
{
    public Vector3[] position_array;
    public Vector3[] rotation_array;
    public float[] time_array;

    public string file_name;
    public string json_string;

    public static TrajectoryInfo CreateFromJSON(string jsonString)
    {
        //Debug.Log("Reading json");
        //return JsonConvert.DeserializeObject<TrajectoryInfo>(jsonString);
        return JsonUtility.FromJson<TrajectoryInfo>(jsonString);
    }

    public void SaveToString()
    {
        json_string = JsonUtility.ToJson(this); // updated 2022
    }

    public void WriteDataToFile()
    {
        SaveToString();
        string path = Application.dataPath + "/Resources/Text/" + file_name;
        //Debug.Log("AssetPath:" + path);
        System.IO.File.WriteAllText(path, json_string);
#if UNITY_EDITOR
        UnityEditor.AssetDatabase.Refresh();
#endif
    }


}

//[System.Serializable]
//public class TrajectoryInfo_old
//{
//    public Vector3[] position_array;
//    public Vector3[] rotation_array;
//    public float[] time_array;

//    public string file_name;
//    public string json_string;

//    public static TrajectoryInfo CreateFromJSON(string jsonString)
//    {
//        //Debug.Log("Reading json");
//        return JsonConvert.DeserializeObject<TrajectoryInfo>(jsonString);
//    }

//    public void SaveToString()
//    {
//        json_string = JsonConvert.SerializeObject(this);
//    }

//    public void WriteDataToFile()
//    {
//        SaveToString();
//        string path = Application.dataPath + "/Resources/Text/" + file_name;
//        //Debug.Log("AssetPath:" + path);
//        System.IO.File.WriteAllText(path, json_string);
//#if UNITY_EDITOR
//        UnityEditor.AssetDatabase.Refresh();
//#endif
//    }


//}