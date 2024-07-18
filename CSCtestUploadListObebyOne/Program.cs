﻿using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Azure;
using Azure.DigitalTwins.Core;
using Azure.Identity;
//using Newtonsoft.Json.Linq;
using System;
using System.Net; // For client side connection to omniverse
using System.Net.Sockets; // For server side socket connection with the Robot controller
using System.Threading.Tasks;
using System.IO;
using System.Collections.Generic;
using System.Text;
using System.Text.Json; // For deserialization of the dtdl description of the twin instances
 //  and for deserialization of operational data to objects where we don't have a defined type
using System.Timers;
class Program
{
    static DigitalTwinsClient m_azureClient;
    static async Task Main(string[] args)
    {
        // 
         string adtInstanceUrl = "https://FrankaMyJoghurtDTCreation.api.weu.digitaltwins.azure.net";
            // Authenticate with the Azure service
        var credential = new DefaultAzureCredential(new DefaultAzureCredentialOptions { ExcludeSharedTokenCacheCredential = true });
        DigitalTwinsClient client = new DigitalTwinsClient(new Uri(adtInstanceUrl), credential);
        m_azureClient = client;
        Console.WriteLine("Service client succsessfully created.");
        List<string> list1 = new List<string> {
                  "0.843742", "0.628378", "0.925081", "0.546127",
                    "0.900415", "0.407821", "0.731900", "0.439281", "0.351066" };
        Random random = new Random();
    

        // 
        List<string> list2 = new List<string>();
        List<string> updateTwinIds = new List<string>{"JointPosition1", "JointPosition2", "JointPosition3", "JointPosition4", "JointPosition5", "JointPosition6", "JointPosition7",
                                                             "GripperData_Width", "GripperData_Speed"};
        // 
      
        for(int i=0; i<updateTwinIds.Count; i++)
        {
            string twinId = updateTwinIds[i];
            string value = list1[i];
            var patch = new JsonPatchDocument();
            patch.AppendAdd("/value", value);
            await m_azureClient.UpdateDigitalTwinAsync(twinId, patch);

            //m_azureClient.UpdateComponent(updateTwinIdswinIds[i], "value", list1[i]); 
            Response<BasicDigitalTwin> getTwinResponse = await m_azureClient.GetDigitalTwinAsync<BasicDigitalTwin>(twinId);
            var twinValue = getTwinResponse.Value;

            string componentValueResult = twinValue.Contents["value"].ToString();
            list2.Add(componentValueResult);
              

            }
            Console.WriteLine("list1 is"+list1);
           Console.WriteLine("List1:");
        foreach (var item in list1)
        {
            Console.WriteLine(item);
        }
            Console.WriteLine("list2 is"+list2);
             Console.WriteLine("List2:");
        foreach (var item in list2)
        {
            Console.WriteLine(item);
        }
        
    }
}
