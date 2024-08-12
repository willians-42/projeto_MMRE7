/*
 * File:   main.c
 * Author: Willians Fernando de Oliveira / Matheus Jose Ferreira Borelli
 * Created on 27 de Julho de 2024, 16:54
 */

// define a frequência em 48MHz
#define _XTAL_FREQ 48000000 

// inclusao de bibliotecas
#include <xc.h>
#include "config_4520.h"
#include "LCD_4520.h"
#include <stdio.h>
#include <string.h>

// botoes
#define BOTAO_CAFE_CURTO     PORTAbits.RA0
#define BOTAO_CAFE_LONGO     PORTAbits.RA1
#define BOTAO_CAFE_COM_LEITE PORTAbits.RA2
#define BOTAO_CAPPUCCINO     PORTAbits.RA3
#define BOTAO_CHOCOLATE      PORTAbits.RA4
#define BOTAO_CHOCOLEITE     PORTAbits.RA5

// sensor
#define SENSOR_CAPACITIVO PORTCbits.RC0

// indicadores
#define AQUECIMENTO_TERMOBLOCO PORTBbits.RB7

void setup(void){
    // componentes
    TRISB = 0x00; // configura todos os pinos PORTB (leds indicadores) como saida (0x00 = 00000000)
    LATB  = 0x00;
    // display
    TRISD = 0x00; //RD0 a RD7 - saída para o LCD
    PORTD = 0x00; //Coloca portD em 0V.
    lcd_inicia(0x28, 0x0f, 0x06); //lnicializa o display LCD alfanumérico com quatrolinhas de dados.
    
    // sensor capacitivo
    TRISCbits.RC0 = 1;
    SENSOR_CAPACITIVO = 0;

    TRISA = 0xFF; // configura todos os pinos de PORTA (botoes) como entrada (0xFF = 11111111)
    ADCON1 = 0b00001111;
    PORTA = 0xFF;
}
   // Função para criar um delay em milisegundos
void delayMilisegundos(unsigned int milisegundos) {
    unsigned int i;

    // Configuração do Timer1
    T1CON = 0x00; // Desliga o Timer1 e limpa o registro de controle
    T1CONbits.TMR1CS = 0; // Seleciona o oscilador interno (Fosc/4)
    T1CONbits.T1CKPS = 0b11; // Prescaler de 1:8

    // Calcular o preload para 1 milissegundo
    // Timer1 overflow ocorre a cada 65536 ciclos de clock
    // Fosc = 48MHz, então Fosc/4 = 12MHz, Prescaler 1:8
    // Timer1 incrementa a cada 0,6667µs (8 / 12MHz)
    // Para 1 milissegundo (1.000µs), precisamos de 1.000µs / 0,6667µs ? 1500 ciclos de timer
    // Como Timer1 conta até 65536, precisamos de múltiplos ciclos de Timer1 para chegar a 1 milissegundo
    // Preload = 65536 - 1500

    // Simplificando o cálculo
    unsigned int overflowCount = 1; // Número de overflows necessários para 1ms
    unsigned int preload = 65536 - 1500; // Preload calculado para 1ms

    for (i = 0; i < milisegundos; i++) {
        for (unsigned int j = 0; j < overflowCount; j++) {
            TMR1H = (preload >> 8) & 0xFF; // Carregar valor alto do Timer1
            TMR1L = preload & 0xFF; // Carregar valor baixo do Timer1

            PIR1bits.TMR1IF = 0; // Limpa a flag de interrupção do Timer1
            T1CONbits.TMR1ON = 1; // Liga o Timer1

            while (!PIR1bits.TMR1IF); // Espera o overflow do Timer1

            T1CONbits.TMR1ON = 0; // Desliga o Timer1
        }
    }
}

void preAquecimento(void){
    lcd_LD_cursor(0); // inibe a exibicao do cursor
    lcd_posicao (1,1);// desloca o cursor para a posicao determinada
    imprime_string_lcd("Em aquecimento"); //envia String para o Display LCD 
    lcd_posicao (2,1);// desloca o cursor para a posicao determinada
    imprime_string_lcd("Aguarde..."); //envia String para o Display LCD
    delayMilisegundos(8000);
}

void maquinaPronta(void){
    lcd_LD_cursor(0); // inibe a exibicao do cursor
    lcd_posicao (1,1);// desloca o cursor para a posicao determinada
    imprime_string_lcd("Coloque o copo e"); //envia String para o Display LCD 
    lcd_posicao (2,1);// desloca o cursor para a posicao determinada
    imprime_string_lcd("escolha a bebida"); //envia String para o Display LCD
}

void bebidaPronta(void){
    lcd_limpa_tela();
    lcd_LD_cursor(0); // inibe a exibicao do cursor
    lcd_posicao (1,1);// desloca o cursor para a posicao determinada
    imprime_string_lcd("Bebida pronta"); //envia String para o Display LCD 
    lcd_posicao (2,1);// desloca o cursor para a posicao determinada
    imprime_string_lcd("Sirva-se!"); //envia String para o Display LCD
    delayMilisegundos(3000);  
}

void selecaoBebidas(int num_bebida){
    char str[25]; // variavel char para armazenar texto
    switch (num_bebida) {
        case 0:
            strcpy(str, "Cafe Curto");
            break;
        case 1:
            strcpy(str, "Cafe Longo");
            break;
        case 2:
            strcpy(str, "Cafe com Leite");
            break;
        case 3:
            strcpy(str, "Cappuccino");
            break;
        case 4:
            strcpy(str, "Chocolate");
            break;
        case 5:  
            strcpy(str, "Chocoleite");
            break;
        default:
            break;  
    }
    lcd_limpa_tela();
    lcd_LD_cursor(0); // inibe a exibicao do cursor
    lcd_posicao (1,1);// desloca o cursor para a posicao determinada
    imprime_string_lcd(str); //envia String para o Display LCD 
    lcd_posicao (2,1);// desloca o cursor para a posicao determinada
    imprime_string_lcd("Preparando..."); //envia String para o Display LCD 
}

void cafeCurto(){
    PORTB = 0b11001010; // entrada de agua - direita, mixer - direita, molas de dosagem - cafe
    delayMilisegundos(10000);  
    PORTB = 0b10001010; // 10 segundos -  cessa mola de dosagem
    delayMilisegundos(2000); // 
    PORTB = 0b10001000; // 12 segundos - cessa entrada de agua
    PORTB = 0b10000000; // encerrado
}

void cafeLongo(){
    PORTB = 0b11001010; // entrada de agua - direita, mixer - direta, molas de dosagem - cafe
    delayMilisegundos(20000); 
    PORTB = 0b10001010; // 20 segundos -  cessa mola de dosagem
    delayMilisegundos(1500); 
    PORTB = 0b10001000; // 21.5 segundos - cessa entrada de agua 
    PORTB = 0b10000000; // encerrado 
}

void cafeComLeite(){
    // CAFE
    PORTB = 0b11101111; // entrada de agua - direita e esquerda, mixer - esquerda e direita, molas de dosagem - cafe e leite
    delayMilisegundos(4000); 
    PORTB = 0b10101111; // cessa mola de dosagem - cafe 
    delayMilisegundos(3000); // 
    PORTB = 0b10100101;  // cessa entrada de agua - direita, mixer - direita
    // LEITE
    delayMilisegundos(5000); 
    PORTB = 0b10000101; // cessa mola de dosagem - leite
    delayMilisegundos(3000); 
    PORTB = 0b10000100; // cessa entrada de agua - esquerda
    PORTB = 0b10000000; // encerrado   
}

void cappuccino(){
    // CAFE
    PORTB = 0b11111111; // entrada de agua - direita e esquerda, mixer - esquerda e direita, molas de dosagem - cafe, leite e chocolate
    delayMilisegundos(4000); 
    PORTB = 0b10111111; // cessa mola de dosagem - cafe 
    delayMilisegundos(3000); // 
    PORTB = 0b10110101;  // cessa entrada de agua - direita, mixer - direita
    // CHOCOLATE
    delayMilisegundos(1000);
    PORTB = 0b10100101; // cessa mola de dosagem - chocolate
    // LEITE
    delayMilisegundos(4000); 
    PORTB = 0b10000101; // cessa mola de dosagem - leite
    delayMilisegundos(2000); 
    PORTB = 0b10000100; // cessa entrada de agua - esquerda
    PORTB = 0b10000000; // encerrado        
}

void chocolate(){
    PORTB = 0b10110101; // entrada de agua - esquerda, mixer - esquerda, molas de dosagem - chocolate, molas de dosagem - leite
    delayMilisegundos(10000); 
    PORTB = 0b10010101; // 10 segundos - cessa mola de dosagem - leite
    delayMilisegundos(5000); 
    PORTB = 0b10000101; // 20 segundos -  cessa mola de dosagem - chocolate
    delayMilisegundos(6000); 
    PORTB = 0b10000100; // 21 segundos - cessa entrada de agua - esquerda
    PORTB = 0b10000000; // encerrado  
}

void chocoleite(){
    PORTB = 0b10110101; // entrada de agua - esquerda, mixer - esquerda, molas de dosagem - chocolate e leite
    delayMilisegundos(15000); // 
    PORTB = 0b10000101; // 15 segundos - cessa molas de dosagem - chocolate e leite
    delayMilisegundos(5000); // 
    PORTB = 0b10000100; // 20 segundos - cessa entrada de agua - esquerda
    delayMilisegundos(250); // 
    PORTB = 0b10000000; // encerrado 
}

void prepararBebida(int num_bebida){
    switch (num_bebida) {
        case 0:
            cafeCurto();
            break;
        case 1:
            cafeLongo();
            break;
        case 2:
            cafeComLeite();
            break;
        case 3:
            cappuccino();
            break;
        case 4:
            chocolate();
            break;
        case 5:  
            chocoleite();
            break;
        default:
            break;  
    }
}

void main(void) {
    setup();
    preAquecimento();
    int estado = 0;
    int escolha = -1;
    while(1){
        AQUECIMENTO_TERMOBLOCO = 1;
        maquinaPronta();
        if (SENSOR_CAPACITIVO == 1){
            if (BOTAO_CAFE_CURTO == 0 && estado == 0){
                escolha = 0;
                estado = 1;
                delayMilisegundos(100); // espera um pouco, trata o debounce
            } else if (BOTAO_CAFE_LONGO == 0 && estado == 0) {
                escolha = 1;
                estado = 1;
                delayMilisegundos(100); 
            } else if (BOTAO_CAFE_COM_LEITE == 0 && estado == 0) {
                escolha = 2;
                estado = 1;
                delayMilisegundos(100); 
            } else if (BOTAO_CAPPUCCINO == 0 && estado == 0) {
                escolha = 3;
                estado = 1;
                delayMilisegundos(100); 
            } else if (BOTAO_CHOCOLATE == 0 && estado == 0) {
                escolha = 4;
                estado = 1;
                delayMilisegundos(100); 
            } else if (BOTAO_CHOCOLEITE == 0 && estado == 0) {
                escolha = 5;
                estado = 1 ;
                delayMilisegundos(100); 
            } 
            if ( estado == 1 && escolha != -1 ){
                selecaoBebidas(escolha);
                prepararBebida(escolha);
                bebidaPronta();
                
                while(SENSOR_CAPACITIVO == 1){} // espera copo ser retirado
                SENSOR_CAPACITIVO = 0;
                estado = 0;
                escolha = -1;
            }
        } 
    }   
    return;
}
