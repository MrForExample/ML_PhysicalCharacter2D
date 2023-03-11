#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;

public class EditorFunctions : MonoBehaviour
{
    // Press N in sence when all sprite selected and select Editor Function Script last
    [Space(5)][Header("Copy Env prefab alone the Z axis")]
    public GameObject envPrefab;
    public int totalNum = 12;
    public float interval = 5f;
    public void DuplicateEnv()
    {
        for (int i = totalNum - 1; i > 0; i--)
        {
            float nowZ = envPrefab.transform.position.z + interval * i;
            Vector3 newPos = new Vector3(envPrefab.transform.position.x, envPrefab.transform.position.y, nowZ);
            var newEnv = PrefabUtility.InstantiatePrefab(envPrefab) as GameObject;
            newEnv.transform.position = newPos;
            newEnv.name += "_" + i;
        }
    }
}

[CustomEditor(typeof(EditorFunctions))]
public class EditorFunctionEditor: Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        EditorFunctions functions = (EditorFunctions)target;

        if(GUILayout.Button("Copy Env prefab alone the Z axis"))
        {
            functions.DuplicateEnv();
        }
    }
    /// <summary>
    /// Need turn on Gizmos in editor
    /// </summary>
    public void OnSceneGUI() 
    {

    }
}
#endif