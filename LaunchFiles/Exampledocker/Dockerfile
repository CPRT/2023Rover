FROM ubuntu:latest

# Name this initial stage "builder"
FROM ubuntu:latest AS builder
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get -y update && apt-get install -y
RUN apt-get -y install g++ cmake git
COPY . .
WORKDIR .
RUN g++ -o  helloworld test.cpp
# Create a new stage for our output container
FROM ubuntu:latest
WORKDIR .
# Copy the built binary over from previous stage
COPY helloworld .
# Run the binary in the new container
CMD ["./helloworld"]
