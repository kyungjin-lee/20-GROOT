//
//  FirstViewController.swift
//  GrootPlayer
//
//  Created by Kyungjin Lee on 2020/06/10.
//  Copyright © 2020 Kyungjin Lee. All rights reserved.
//

import UIKit

class FirstViewController: UIViewController{
      
       
    @IBOutlet weak var userName: UITextField!
    @IBOutlet weak var startButton: UIButton!
    
 
    @IBOutlet weak var isSaveViewTraceSwitch: UISwitch!
    
    @IBOutlet weak var initialDepthField: UITextField!
    
    @IBOutlet weak var datasetNameField: UITextField!
    
    override func viewDidLoad() {
           super.viewDidLoad()

           
           // Do any additional setup after loading the view.
       }
       
       
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        guard let secondVC = segue.destination as? ViewController else {return}
        secondVC.userNameValue = userName.text!
        secondVC.datasetName = datasetNameField.text!
        secondVC.initialDepth = initialDepthField.text!
        if(isSaveViewTraceSwitch.isOn)
        {
            secondVC.isSaveViewTrace = true
            
        }
    }
    
    override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent?) {
                    view.endEditing(true)
                }
                func textFieldShouldReturn(_ textField: UITextField) -> Bool {
                     //All the textFields in the form
                  let textFields = [userName, datasetNameField, initialDepthField]
                  let firstResponder = textFields.first(where: {$0?.isFirstResponder ?? false })
                  firstResponder??.resignFirstResponder()
                    return true
                }
}
