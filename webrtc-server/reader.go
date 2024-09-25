package main

import (
	"log"
	"os"
)

type FileData struct {
	Name string
	Data []byte
}

func ReadBinaryFiles(contentDirectory string) ([]FileData, error) {
	var fileData []FileData
	files, err := os.ReadDir(contentDirectory)
	if err != nil {
		log.Fatal(err)
	}
	for _, f := range files {
		data, err := os.ReadFile(contentDirectory + "/" + f.Name())
		if err != nil {
			log.Printf("Failed to read file %s", f.Name())
			return nil, err
		}

		fileData = append(fileData, FileData{
			Name: f.Name(),
			Data: data,
		})
	}
	return fileData, nil
}
