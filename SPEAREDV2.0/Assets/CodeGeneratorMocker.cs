using Assets.LejosTemplates;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CodeGeneratorMocker : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        LejosMainCodeGen codeGen = new LejosMainCodeGen();
        codeGen.SetPackageName("test2");
        codeGen.SetProgramName("HalloWelt");
        Program p = new Program();
        p.statements = new List<Statement>();
        var newMove = new LejosMoveStatement();
        newMove.x = 20;
        newMove.z = 20;
        p.statements.Add(newMove);
        codeGen.SetProgram(p);
        var compiler = GetComponent<LejosCompileRequester>();
        StartCoroutine(compiler.RequestCompilation(codeGen.TransformText()));
        compiler.compiled.AddListener((result) =>
        {
            Debug.Log("Compiled"+result.Length);
        });

    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
