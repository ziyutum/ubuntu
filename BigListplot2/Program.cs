﻿using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Azure;
using Azure.DigitalTwins.Core;
using Azure.Identity;
using System.Text.Json;
using System.Globalization;
using System.IO;

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

        // 
        DateTime programStart = DateTime.UtcNow;

        // 
        List<List<string>> list1 = new List<List<string>>();
        Random random = new Random();
        for (int i = 0; i < 50; i++)
        {
            List<string> values = new List<string>();
            for (int j = 0; j < 9; j++)
            {
                values.Add(random.NextDouble().ToString("F6"));
            }
            list1.Add(values);
        }

        // 
        List<string> updateTwinIds = new List<string>
        {
            "JointPosition1", "JointPosition2", "JointPosition3", "JointPosition4", "JointPosition5", "JointPosition6", "JointPosition7",
            "GripperData_Width", "GripperData_Speed"
        };

        using (StreamWriter sw = new StreamWriter("delays_with_timestamps.csv"))
        {
            sw.WriteLine("Group,Upload Start Timestamp,Upload End Timestamp,Upload Delay (ms),Download Start Timestamp,Download End Timestamp,Download Delay (ms),Total Elapsed Time (ms)");

            for (int i = 0; i < list1.Count; i++)
            {
                List<string> valuesToSend = list1[i];

                // UPLOAD TIMESTAMP
        
                string uploadStartTimestamp = DateTime.UtcNow.ToString("yyyy-MM-dd HH:mm:ss.ffffff");
                DateTime uploadStartTime = DateTime.UtcNow;

                // 
                for (int j = 0; j < valuesToSend.Count; j++)
                {
                    string twinId = updateTwinIds[j % updateTwinIds.Count];
                    string value = valuesToSend[j];

                    var patch = new JsonPatchDocument();
                    patch.AppendAdd("/value", value);
                    await m_azureClient.UpdateDigitalTwinAsync(twinId, patch);
                }

                // 
                string uploadEndTimestamp = DateTime.UtcNow.ToString("yyyy-MM-dd HH:mm:ss.ffffff");
                DateTime uploadEndTime = DateTime.UtcNow;

                // DOWNLOADING
                string downloadStartTimestamp = DateTime.UtcNow.ToString("yyyy-MM-dd HH:mm:ss.ffffff");
                DateTime downloadStartTime = DateTime.UtcNow;

                // 
                for (int j = 0; j < valuesToSend.Count; j++)
                {
                    string twinId = updateTwinIds[j % updateTwinIds.Count];
                    Response<BasicDigitalTwin> getTwinResponse = await m_azureClient.GetDigitalTwinAsync<BasicDigitalTwin>(twinId);
                    var twinValue = getTwinResponse.Value;
                    string componentValueResult = twinValue.Contents["value"].ToString();
                }

                // 
                string downloadEndTimestamp = DateTime.UtcNow.ToString("yyyy-MM-dd HH:mm:ss.ffffff");
                DateTime downloadEndTime = DateTime.UtcNow;

                // 
                double uploadDelay = (uploadEndTime - uploadStartTime).TotalMilliseconds;
                double downloadDelay = (downloadEndTime - downloadStartTime).TotalMilliseconds;

                // 
                double totalElapsedTime = (downloadEndTime - programStart).TotalMilliseconds;

                // 
                sw.WriteLine($"{i + 1},{uploadStartTimestamp},{uploadEndTimestamp},{uploadDelay},{downloadStartTimestamp},{downloadEndTimestamp},{downloadDelay},{totalElapsedTime}");

                Console.WriteLine($"Group {i + 1} processed: Upload Delay = {uploadDelay} ms, Download Delay = {downloadDelay} ms, Total Elapsed Time = {totalElapsedTime} ms");
            }
        }

        //
        DateTime programEnd = DateTime.UtcNow;
        double totalProgramTime = (programEnd - programStart).TotalMilliseconds;
        Console.WriteLine($"Total program execution time: {totalProgramTime} ms");
    }
}
