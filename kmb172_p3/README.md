# kmb172_p3

Extends the P1 functionality again by providing an action server / action client interface for the sin wave commander. The simulator and controller are the same as always. The commander node now acts as an action server, and it is called sin_commander p3. The action client is called command_client3.

I have defined WavCycles.action in order to transmit the data required for this program. It contains a request field where the key information about the wave is placed as a goal. The response and feedback sections have basically token values.

For other detail, see the pdf writeup.

## Example usage

## Running tests/demos
    
