﻿using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Azure;
using Azure.DigitalTwins.Core;
using Azure.Identity;
using System.Text.Json;

class Program
{
    static DigitalTwinsClient m_azureClient;
    
    static async Task Main(string[] args)
    {
        string adtInstanceUrl = "https://FrankaMyJoghurtDTCreation.api.weu.digitaltwins.azure.net";
        var credential = new DefaultAzureCredential(new DefaultAzureCredentialOptions { ExcludeSharedTokenCacheCredential = true });
        DigitalTwinsClient client = new DigitalTwinsClient(new Uri(adtInstanceUrl), credential);
        m_azureClient = client;
        
        Console.WriteLine("Service client successfully created.");
        
        // Generating 5 groups of 9 random values each
        List<List<string>> list1 = new List<List<string>>();
        Random random = new Random();
        for (int i = 0; i < 100; i++)
        {
            List<string> values = new List<string>();
            for (int j = 0; j < 9; j++)
            {
                values.Add(random.NextDouble().ToString("F6"));
            }
            list1.Add(values);
        }

        //print list 1
        Console.WriteLine("List1:");
        foreach (var values in list1)
        {
            Console.WriteLine(string.Join(", ", values));
        }
        // List to store retrieved values
        List<List<string>> list2 = new List<List<string>>();
         List<(string uploadTimestamp, string uploadEndTimestamp, string downloadTimestamp, string downloadEndTimestamp, double uploadDelay, double downloadDelay)> results = new List<(string, string, string, string, double, double)>();


        // Update twin ids
        List<string> updateTwinIds = new List<string>
        {
            "JointPosition1", "JointPosition2", "JointPosition3", "JointPosition4", "JointPosition5","JointPosition6","JointPosition7",
            "GripperData_Width", "GripperData_Speed"
        };

      
        for (int i = 0; i < list1.Count; i++)
        {
            string twinId = updateTwinIds[i % updateTwinIds.Count];
            List<string> valuesToSend = list1[i]; //
            string uploadTimestamp = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.ffffff");
 

            var patch = new JsonPatchDocument();
            patch.AppendAdd("/value", JsonSerializer.Serialize(valuesToSend)); // 
            await m_azureClient.UpdateDigitalTwinAsync(twinId, patch);
            string uploadEndTimestamp = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.ffffff");

            string downloadTimestamp = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.ffffff");

            Response<BasicDigitalTwin> getTwinResponse = await m_azureClient.GetDigitalTwinAsync<BasicDigitalTwin>(twinId);

             var twinValue = getTwinResponse.Value;

            // 
            string componentValueResult = twinValue.Contents["value"].ToString();

            //
            List<string> componentValues = JsonSerializer.Deserialize<List<string>>(componentValueResult);
            string downloadEndTimestamp = DateTime.UtcNow.ToString("yyyy-MM-dd HH:mm:ss.ffffff");
             results.Add((uploadTimestamp, uploadEndTimestamp, downloadTimestamp, downloadEndTimestamp, uploadDelay, downloadDelay));
            using (StreamWriter sw = new StreamWriter("delays_with_timestamps.csv"))
            {
                sw.WriteLine("Upload Start Timestamp,Upload End Timestamp,Upload Delay (ms),Download Start Timestamp,Download End Timestamp,Download Delay (ms)");
                foreach (var result in results)
                {
                    sw.WriteLine($"{result.uploadTimestamp},{result.uploadEndTimestamp},{result.uploadDelay},{result.downloadTimestamp},{result.downloadEndTimestamp},{result.downloadDelay}");
                }
            }

            Console.WriteLine("List2component:"+componentValues);
            foreach (var values in componentValues)
            {
                Console.WriteLine(string.Join(", ", values));
            }
           

        }

        // Print out list1 and list2
     

        // Console.WriteLine("List2:");
        // foreach (var values in list2)
        // {
        //     Console.WriteLine(string.Join(", ", values));
        // }
    }
}
